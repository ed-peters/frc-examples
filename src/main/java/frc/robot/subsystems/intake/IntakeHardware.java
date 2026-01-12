package frc.robot.subsystems.intake;

/**
 * Interface for the hardware of an arm subsystem
 */
public interface IntakeHardware {

    /** @return is the motor brake enabled? */
    boolean isBrakeEnabled();

    /** Enables/disables the motor brake */
    void setBrake(boolean brake);

    /** @return current motor speed in revolutions per second */
    double getMotorSpeed();

    /** @return motor output current in amps */
    double getMotorAmps();

    /** Applies voltage to the motor */
    void applyVolts(double volts);

}
