package frc.robot.subsystems.arm;

/**
 * Interface for the hardware of an arm subsystem
 */
public interface ArmHardware {

    /** @return is the motor brake enabled? */
    boolean isBrakeEnabled();

    /** Enables/disables the motor brake */
    void setBrake(boolean brake);

    /**
     * @return angle of the arm in degrees (this probably comes from an
     * absolute encoder)
     */
    double getAngle();

    /** @return arm velocity in degrees per second */
    double getVelocity();

    /** @return motor output current in amps */
    double getMotorAmps();

    /** Applies voltage to the motor */
    void applyVolts(double volts);

}
