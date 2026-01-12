package frc.robot.util;

/**
 * <p>Represents the speed of a wheel in three ways: the motor speed (rps at
 * the motor shaft), the wheel speed (including gear ratio) and the linear
 * speed of the wheel surface (including wheel circumference).</p>
 *
 * <p>Allows you to set any one of the values and compute the others
 * automatically from configuration.</p>
 */
public class WheelSpeed {

    final double gearRatio;
    final double wheelCircumference;
    double driveSpeed;
    double wheelSpeed;
    double linearSpeed;

    public WheelSpeed(double gearRatio, double wheelCircumference) {
        this.gearRatio = gearRatio;
        this.wheelCircumference = wheelCircumference;
        this.driveSpeed = Double.NaN;
        this.wheelSpeed = Double.NaN;
        this.linearSpeed = Double.NaN;
    }

    public boolean hasSpeed() {
        return Double.isFinite(driveSpeed);
    }

    /**
     * @return drive speed
     */
    public double getDriveSpeed() {
        return driveSpeed;
    }

    /**
     * @return rotational velocity of the wheel
     */
    public double getWheelSpeed() {
        return wheelSpeed;
    }

    /**
     * @return linear speed of the surface of the wheel
     */
    public double getLinearSpeed() {
        return linearSpeed;
    }

    /**
     * Sets the motor speed, and derives other values from it
     * @param motorSpeed new drive speed
     */
    public void setMotorSpeed(double motorSpeed) {
        this.driveSpeed = motorSpeed;
        this.wheelSpeed = motorSpeed * gearRatio;
        this.linearSpeed = motorSpeed * wheelCircumference;
    }

    /**
     * Sets the wheel speed, and derives other values from it
     * @param wheelSpeed new wheel speed
     */
    public void setWheelSpeed(double wheelSpeed) {
        this.wheelSpeed = wheelSpeed;
        this.driveSpeed = wheelSpeed / gearRatio;
        this.linearSpeed = wheelSpeed * wheelCircumference;
    }

    /**
     * Sets the linear speed, and derives other values from it
     * @param linearSpeed new drive speed
     */
    public void setLinearSpeed(double linearSpeed) {
        this.linearSpeed = linearSpeed;
        this.wheelSpeed = linearSpeed / wheelCircumference;
        this.driveSpeed = wheelSpeed / gearRatio;
    }

    /**
     * Clears all values
     */
    public void clear() {
        this.driveSpeed = Double.NaN;
        this.wheelSpeed = Double.NaN;
        this.linearSpeed = Double.NaN;
    }
}
