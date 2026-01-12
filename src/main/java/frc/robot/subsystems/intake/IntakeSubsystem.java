package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.PDController;
import frc.robot.util.Util;
import frc.robot.util.WheelSpeed;

import java.util.Objects;
import java.util.function.DoubleSupplier;

import static frc.robot.Config.Intake.ejectSpeed;
import static frc.robot.Config.Intake.gearRatio;
import static frc.robot.Config.Intake.gobbleSpeed;
import static frc.robot.Config.Intake.indexSpeed;
import static frc.robot.Config.Intake.p;
import static frc.robot.Config.Intake.tolerance;
import static frc.robot.Config.Intake.v;
import static frc.robot.Config.Intake.wheelCircumference;

/**
 * Implementation of a subsystem that manages an intake. This is a motor with
 * an attached gearbox wheel, which knows how to run at a fixed rate (either
 * wheel revolutions per second or linear feet per second). PID control is
 * done in software.
 */
public class IntakeSubsystem extends SubsystemBase {

    /**
     * Setting this to true will make this subsystem publish low-level voltage
     * and current information
     */
    static final boolean verboseLogging = true;

    /**
     * Represents a preset linear speed for the arm, with a value derived from
     * configuration
     */
    public enum IntakePreset {

        GOBBLE,
        INDEX,
        EJECT;

        /**
         * @return the linear speed corresponding to this preset in feet per
         * second
         */
        public double linearSpeed() {
            DoubleSupplier speed = switch (this) {
                case GOBBLE -> gobbleSpeed;
                case INDEX -> indexSpeed;
                case EJECT -> () -> -Math.abs(ejectSpeed.getAsDouble());
            };
            return speed.getAsDouble();
        }
    }

//region Implementation --------------------------------------------------------

    final IntakeHardware hardware;
    final PDController pid;
    final WheelSpeed currentSpeed;
    final WheelSpeed setpoint;
    String currentMode;
    double latestFeedback;
    double latestFeedforward;
    double latestVolts;

    /**
     * Creates a {@link IntakeSubsystem}
     * @param hardware the associated hardware (required)
     * @throws IllegalArgumentException if required parameters are null
     */
    public IntakeSubsystem(IntakeHardware hardware) {

        this.hardware = Objects.requireNonNull(hardware);
        this.pid = new PDController(p, () -> 0.0, tolerance);
        this.currentSpeed = new WheelSpeed(gearRatio, wheelCircumference);
        this.setpoint = new WheelSpeed(gearRatio, wheelCircumference);
        this.currentMode = "idle";

        SmartDashboard.putData(getName(), builder -> {
            builder.addBooleanProperty("AtGoal?", this::atTarget, null);
            builder.addStringProperty("Mode", () -> currentMode, null);
            builder.addDoubleProperty("MotorSpeedCurrent", currentSpeed::getDriveSpeed, null);
            builder.addDoubleProperty("LinearSpeedCurrent", currentSpeed::getLinearSpeed, null);
            builder.addDoubleProperty("WheelSpeedCurrent", currentSpeed::getWheelSpeed, null);
            builder.addDoubleProperty("WheelSpeedError", pid::getError, null);
            builder.addDoubleProperty("WheelSpeedTarget", setpoint::getWheelSpeed, null);
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
     * can be used to create Triggers for shooting velocities for example)
     */
    public boolean atTarget() {
        return setpoint.hasSpeed() && pid.atSetpoint();
    }

    /**
     * @param rps a wheel speed in revolutions per second
     * @return are we currently at the specified wheel speed, within the
     * configured tolerance?
     */
    public boolean atWheelSpeed(double rps) {
        return Math.abs(currentSpeed.getWheelSpeed() - rps) < tolerance.getAsDouble();
    }

    @Override
    public void periodic() {

        // the only thing we do every period is refresh our current velocity
        // calculations based on the underlying motor velocity
        currentSpeed.setMotorSpeed(hardware.getMotorSpeed());
    }

    /*
     * Runs the wheel in "open loop" mode, clearing the setpoint and supplying
     * a raw input voltage
     */
    private void openLoop(double volts) {
        setpoint.clear();
        latestFeedforward = Double.NaN;
        latestFeedback = Double.NaN;
        latestVolts = Util.clampVolts(volts);
        hardware.applyVolts(latestVolts);
    }

    /*
     * Runs the wheel in "open loop" mode, calculating feedback and feedforward
     * from the current setpoint
     */
    private void closedLoop() {
        latestFeedforward = v.getAsDouble() * setpoint.getWheelSpeed();
        latestFeedback = pid.calculate(
                currentSpeed.getWheelSpeed(),
                setpoint.getWheelSpeed());
        latestVolts = Util.clampVolts(latestFeedforward + latestFeedback);
        hardware.applyVolts(latestVolts);
    }

//endregion

//region Command factories -----------------------------------------------------

    /**
     * @return a command that "idles" the wheel by supplying 0 volts (this is
     * probably a good candidate for a default command)
     */
    public Command idleCommand() {
        // idling is just running forever at 0 volts
        return startRun(
                () -> {
                    currentMode = "idle";
                    setpoint.clear();
                },
                () -> openLoop(0.0));
    }

    /**
     * @param input a supplier of input for teleop (-1.0 - 1.0)
     * @return a command that will run the wheel in open loop mode based on
     * the supplied input (note that this is probably only good for testing
     * - it's hard to control a wheel accurately in teleop)
     */
    public Command teleopCommand(DoubleSupplier input) {
        return startRun(
                () -> {
                    currentMode = "teleop";
                    setpoint.clear();
                },
                () -> {
                    openLoop(input.getAsDouble() * Util.MAX_VOLTS);
                });
    }

    /**
     * @param mode the name of the mode for this setting
     * @param rps the desired wheel revolutions per second
     * @return a command that will move the wheel at that speed
     */
    public Command wheelSpeedCommand(String mode, double rps) {
        return startRun(
                () -> {
                    currentMode = mode;
                    setpoint.setWheelSpeed(rps);
                    pid.reset();
                },
                this::closedLoop);
    }

    /**
     * @param mode the name of the mode for this setting
     * @param fps the desired linear speed in feet per second
     * @return a command that will move the wheel at that speed
     */
    public Command linearSpeedCommand(String mode, double fps) {
        return startRun(
                () -> {
                    currentMode = mode;
                    setpoint.setLinearSpeed(fps);
                    pid.reset();
                },
                this::closedLoop);
    }

    /**
     * @param preset a {@link IntakePreset}
     * @return a command that will move the wheel at that speed
     */
    public Command presetCommand(IntakePreset preset) {

        Objects.requireNonNull(preset);

        // look familiar? also the same as RPS. note that we interpret the
        // preset speed as feet per second
        return startRun(
                () -> {
                    currentMode = preset.name();
                    setpoint.setLinearSpeed(preset.linearSpeed());
                },
                this::closedLoop);
    }

//endregion

}
