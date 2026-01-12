package frc.robot.subsystems.arm;

import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.PDController;
import frc.robot.util.RateCalculator;
import frc.robot.util.Trapezoid;
import frc.robot.util.Util;

import java.util.Objects;
import java.util.function.DoubleSupplier;

import static frc.robot.Config.Arm.angleHigh;
import static frc.robot.Config.Arm.angleLow;
import static frc.robot.Config.Arm.angleMiddle;
import static frc.robot.Config.Arm.d;
import static frc.robot.Config.Arm.g;
import static frc.robot.Config.Arm.maxAcceleration;
import static frc.robot.Config.Arm.maxAngle;
import static frc.robot.Config.Arm.maxFeedback;
import static frc.robot.Config.Arm.maxVelocity;
import static frc.robot.Config.Arm.minAngle;
import static frc.robot.Config.Arm.p;
import static frc.robot.Config.Arm.tolerance;
import static frc.robot.Config.Arm.v;

/**
 * Implementation of a subsystem that controls a single-jointed arm.
 */
public class ArmSubsystem extends SubsystemBase {

    /**
     * Setting this to true will make this subsystem publish low-level voltage
     * and current information
     */
    static final boolean verboseLogging = true;

    /**
     * Represents a preset angle for the arm, with an angle value derived from
     * configuration
     */
    public enum ArmPreset {

        HIGH,
        MIDDLE,
        LOW;

        /** @return the angle corresponding to this preset in degrees */
        public double angle() {
            return switch (this) {
                case HIGH -> angleHigh.getAsDouble();
                case MIDDLE -> angleMiddle.getAsDouble();
                case LOW -> angleLow.getAsDouble();
            };
        }
    }

//region Implementation --------------------------------------------------------

    final ArmHardware hardware;
    final ArmSetpoint setpoint;
    final PDController pid;
    final RateCalculator velocityCalculator;
    String currentMode;
    double currentAngle;
    double currentVelocity;
    double latestFeedback;
    double latestFeedforward;
    double latestVolts;

    /**
     * Creates a {@link ArmSubsystem}
     * @param hardware the hardware (required)
     * @throws IllegalArgumentException if required parameters are null
     */
    public ArmSubsystem(ArmHardware hardware) {

        this.hardware = Objects.requireNonNull(hardware);
        this.currentMode = "new";
        this.setpoint = new ArmSetpoint();
        this.pid = new PDController(p, d, tolerance);
        this.velocityCalculator = new RateCalculator(() -> currentAngle);

         SmartDashboard.putData("ArmSubsystem", builder -> {
             builder.addDoubleProperty("AngleCurrent", () -> currentAngle, null);
             builder.addDoubleProperty("AngleGoal", setpoint::getGoalAngle, null);
             builder.addDoubleProperty("AngleTarget", setpoint::getNextAngle, null);
             builder.addBooleanProperty("Brake?", hardware::isBrakeEnabled, hardware::setBrake);
             builder.addBooleanProperty("AtGoal?", this::atGoal, null);
             builder.addDoubleProperty("ErrorAngle", () -> setpoint.nextAngle - currentAngle, null);
             builder.addDoubleProperty("ErrorVelocity", () -> setpoint.nextVelocity - currentVelocity, null);
             builder.addStringProperty("Mode", () -> currentMode, null);
             builder.addDoubleProperty("VelocityCurrent", () -> currentVelocity, null);
             builder.addDoubleProperty("VelocityTarget", setpoint::getNextVelocity, null);
             if (verboseLogging) {
                 builder.addDoubleProperty("Amps", hardware::getMotorAmps, null);
                 builder.addDoubleProperty("VoltsFeedforward", () -> latestFeedforward, null);
                 builder.addDoubleProperty("VoltsFeedback", () -> latestFeedback, null);
                 builder.addDoubleProperty("VoltsTotal", () -> latestVolts, null);
             }
         });
    }

    /**
     * @return do we have a goal and, if so, are we at the goal (this
     * can be used to create Triggers for shooting positions for example)
     */
    public boolean atGoal() {
        return setpoint.hasGoal() && atAngle(setpoint.getGoalAngle());
    }

    /**
     * @param angle an arm angle in degrees
     * @return are we currently at the specified angle, within the
     * configured tolerance?
     */
    public boolean atAngle(double angle) {
        return Math.abs(currentAngle - angle) < tolerance.getAsDouble();
    }

    @Override
    public void periodic() {

        // the only thing we do every period is refresh our current velocity
        // calculations based on the underlying motor velocity
        currentAngle = hardware.getAngle();
        currentVelocity = hardware.getVelocity();
    }

    /*
     * Runs the arm in "open loop" mode, clearing the setpoint and supplying a
     * raw input voltage
     */
    private void openLoop(double volts) {
        latestFeedforward = Double.NaN;
        latestFeedback = Double.NaN;
        latestVolts = Util.clampVolts(volts);
        hardware.applyVolts(latestVolts);
    }

    /*
     * Runs the arm in "open loop" mode, calculating feedback and feedforward
     * from the current setpoint
     */
    private void closedLoop() {
        double na = setpoint.getNextAngle();
        double nv = setpoint.getNextVelocity();
        latestFeedforward = g.getAsDouble() * Math.cos(Math.toRadians(na))
                + v.getAsDouble() * nv;
        latestFeedback = Util.applyClamp(pid.calculate(currentAngle, na), maxFeedback);
        latestVolts = Util.clampVolts(latestFeedforward + latestFeedback);
        hardware.applyVolts(latestVolts);
    }

//endregion

//region Command factories -----------------------------------------------------

    /**
     * @return a command that "idles" the arm by supplying 0 volts (note: this
     * is probably not a good default command - it will send the arm crashing
     * down to the bottom when there is no input supplied)
     */
    public Command idleCommand() {
        return startRun(
                () -> {
                    currentMode = "idle";
                    setpoint.clear();
                },
                () -> openLoop(0.0));
    }

    /**
     * @param input a supplier of input for teleop (-1.0 - 1.0)
     * @return a command that will run the arm in open loop mode based on the
     * supplied input (note that this is probably only good for testing - it's
     * hard to control an arm accurately in teleop)
     */
    public Command teleopCommand(DoubleSupplier input) {
        return startRun(
                () -> {
                    currentMode = "teleop";
                    setpoint.clear();
                },
                () -> openLoop(input.getAsDouble() * Util.MAX_VOLTS));
    }

    /**
     * @return a command that will hold the arm still at the current goal
     * angle, or whatever angle it's currently at if there's no goal
     */
    public Command holdCommand() {
        return startRun(
                () -> {
                    currentMode = "hold";
                    pid.reset();

                    // hard learning from previous years - your preset command
                    // will land you within a tolerance of your goal; if you
                    // just hold still at the "current angle", you might wind
                    // up within 2x the tolerance, which is sloppy. so we use
                    // the goal height here if we have one.
                    setpoint.hold(setpoint.hasGoal()
                            ? setpoint.getGoalAngle()
                            : currentAngle);
                },
                this::closedLoop);
    }

    /**
     * @param velocity a velocity in degrees per second
     * @return a command that will move the arm from its current position at
     * the specified velocity (this is a critical part of tuning the arm)
     */
    public Command constantVelocityCommand(double velocity) {
        return startRun(
                () -> {
                    currentMode = "tuning";
                    pid.reset();
                },
                () -> {
                    double goalAngle = velocity > 0
                            ? maxAngle.getAsDouble()
                            : minAngle.getAsDouble();
                    double nextAngle = currentAngle + velocity * Util.DT;
                    setpoint.set(goalAngle, nextAngle, velocity);
                    closedLoop();
                });
    }

    /**
     * @param preset a preset angle
     * @return a command that will move the arm to that angle using a
     * trapezoid motion profile
     */
    public Command presetCommand(ArmPreset preset) {
        Trapezoid trapezoid = new Trapezoid(maxVelocity, maxAcceleration);
        Timer timer = new Timer();
        return startRun(
                () -> {
                    currentMode = preset.name();
                    pid.reset();
                    trapezoid.calculate(
                            currentAngle,
                            currentVelocity,
                            preset.angle());
                    timer.restart();
                },
                () -> {
                    State state = trapezoid.sample(timer.get());
                    setpoint.set(preset.angle(), state.position, state.velocity);
                    closedLoop();
                });
    }

//endregion

}
