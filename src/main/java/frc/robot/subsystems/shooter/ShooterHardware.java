package frc.robot.subsystems.shooter;

/**
 * Interface for the hardware of an arm subsystem
 */
public interface ShooterHardware {

    /** @return is the motor brake enabled? */
    boolean isBrakeEnabled();

    /** Enables/disables the motor brake */
    void setBrake(boolean brake);

    /** @return current motor speed in revolutions per second */
    double getMotorSpeed();

    /** @return motor output current in amps */
    double getMotorAmps();

    /** Reset the PID error and feedback/feedforward parameters */
    void resetPid(double p, double d, double v);

    /** Runs the motor in closed-loop at the supplied speed */
    void applySpeed(double rps);

    /** Runs the motor in open-loop at the supplied voltage */
    void applyVolts(double volts);
}
