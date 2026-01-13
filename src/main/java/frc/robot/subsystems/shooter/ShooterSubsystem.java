package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.Util;
import frc.robot.util.WheelSpeed;

import java.util.Objects;
import java.util.function.DoubleSupplier;

import static frc.robot.Config.Shooter.d;
import static frc.robot.Config.Shooter.p;
import static frc.robot.Config.Shooter.speed1;
import static frc.robot.Config.Shooter.gearRatio;
import static frc.robot.Config.Shooter.speed2;
import static frc.robot.Config.Shooter.speed3;
import static frc.robot.Config.Shooter.tolerance;
import static frc.robot.Config.Shooter.v;
import static frc.robot.Config.Shooter.wheelCircumference;

/**
 * <p>Implementation of a subsystem that manages a shooter. This is a motor
 * with an attached gearbox and wheel, which knows how to run at a fixed rate
 * (either wheel revolutions per second or linear feet per second).</p>
 *
 * <p>In comparison with the {@link frc.robot.subsystems.intake.IntakeSubsystem},
 * this guy relies on hardware PID control. PID and feedforward parameters are
 * sources from Preferences, and will be reset whenever a closed-loop command
 * is run.</p>
 */
public class ShooterSubsystem extends SubsystemBase {

    /**
     * Setting this to true will make this subsystem publish low-level voltage
     * and current information
     */
    static final boolean verboseLogging = true;

    /**
     * Represents a preset linear speed for the arm, with a value derived from
     * configuration
     */
    public enum ShooterPreset {

        S1,
        S2,
        S3;

        /**
         * @return the linear speed corresponding to this preset in feet per
         * second
         */
        public double linearSpeed() {
            DoubleSupplier speed = switch (this) {
                case S1 -> speed1;
                case S2 -> speed2;
                case S3 -> speed3;
            };
            return speed.getAsDouble();
        }
    }

//region Implementation --------------------------------------------------------

    final ShooterHardware hardware;
    final WheelSpeed currentSpeed;
    final WheelSpeed setpoint;
    String currentMode;
    double latestSpeed;
    double latestVolts;

    /**
     * Creates a {@link ShooterSubsystem}
     * @param hardware the associated hardware (required)
     * @throws IllegalArgumentException if required parameters are null
     */
    public ShooterSubsystem(ShooterHardware hardware) {

        this.hardware = Objects.requireNonNull(hardware);
        this.currentSpeed = new WheelSpeed(gearRatio, wheelCircumference);
        this.setpoint = new WheelSpeed(gearRatio, wheelCircumference);
        this.currentMode = "idle";

        SmartDashboard.putData(getName(), builder -> {
            builder.addBooleanProperty("AtGoal?", this::atTarget, null);
            builder.addStringProperty("Mode", () -> currentMode, null);
            builder.addDoubleProperty("MotorSpeedCurrent", currentSpeed::getDriveSpeed, null);
            builder.addDoubleProperty("LinearSpeedCurrent", currentSpeed::getLinearSpeed, null);
            builder.addDoubleProperty("LinearSpeedError", this::getLinearSpeedError, null);
            builder.addDoubleProperty("LinearSpeedTarget", setpoint::getLinearSpeed, null);
            builder.addDoubleProperty("WheelSpeedCurrent", currentSpeed::getWheelSpeed, null);
            if (verboseLogging) {
                builder.addDoubleProperty("Amps", hardware::getMotorAmps, null);
                builder.addDoubleProperty("VoltsOpenLoop", () -> latestVolts, null);
            }
        });
    }

    /**
     * @return current speed error
     */
    public double getLinearSpeedError() {
        return setpoint.getLinearSpeed() - currentSpeed.getLinearSpeed();
    }

    /**
     * @return do we have a goal and, if so, are we at the goal (this
     * can be used to create Triggers for shooting velocities for example)
     */
    public boolean atTarget() {
        return setpoint.hasSpeed() && atLinearSpeed(setpoint.getLinearSpeed());
    }

    /**
     * @param fps a linear speed in feet per second
     * @return are we currently at the specified linear speed, within the
     * configured tolerance?
     */
    public boolean atLinearSpeed(double fps) {
        return Math.abs(currentSpeed.getLinearSpeed() - fps) < tolerance.getAsDouble();
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
        latestSpeed = Double.NaN;
        latestVolts = Util.clampVolts(volts);
        hardware.applyVolts(latestVolts);
    }

    /*
     * Runs the wheel in "open loop" mode, calculating feedback and feedforward
     * from the current setpoint
     */
    private void closedLoop() {
        latestSpeed = setpoint.getWheelSpeed();
        latestVolts = Double.NaN;
        hardware.applySpeed(latestSpeed);
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
                    hardware.resetPid(p.getAsDouble(), d.getAsDouble(), v.getAsDouble());
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
                    hardware.resetPid(p.getAsDouble(), d.getAsDouble(), v.getAsDouble());
                },
                this::closedLoop);
    }

    /**
     * @param preset a {@link ShooterPreset}
     * @return a command that will move the wheel at that speed
     */
    public Command presetCommand(ShooterPreset preset) {

        Objects.requireNonNull(preset);

        // look familiar? also the same as RPS. note that we interpret the
        // preset speed as feet per second
        return startRun(
                () -> {
                    currentMode = preset.name();
                    setpoint.setLinearSpeed(preset.linearSpeed());
                    hardware.resetPid(p.getAsDouble(), d.getAsDouble(), v.getAsDouble());
                },
                this::closedLoop);
    }

//endregion

}
