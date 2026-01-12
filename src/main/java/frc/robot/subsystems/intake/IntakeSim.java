package frc.robot.subsystems.intake;

import frc.robot.util.Util;

/**
 * Implements {@link IntakeHardware} for simulation. This implementation is
 * a very idealized flywheel.
 */
public class IntakeSim implements IntakeHardware {

    boolean brake = false;
    double velocity = 0.0;
    double position = 0.0;

    @Override
    public boolean isBrakeEnabled() {
        return brake;
    }

    @Override
    public double getMotorSpeed() {
        return velocity;
    }

    @Override
    public double getMotorAmps() {
        return 0;
    }

    @Override
    public void applyVolts(double volts) {
        if (volts == 0.0) {
            if (brake) {
                velocity = 0.0;
            } else {
                velocity = calculateCoasting(velocity);
            }
        } else {
            velocity = calculateSpeed(volts);
        }
        position += velocity * Util.DT;
    }

    @Override
    public void setBrake(boolean brake) {
        this.brake = brake;
    }

    /*
     * Arbitrary "coasting" calculation - we assume speed drops off by 1%
     * every cycle until it gets close to a minimum speed (this could be
     * replaced with a real simulation if you want)
     */
    private double calculateCoasting(double currentVelocity) {
        return Math.min(2.0, currentVelocity * 0.9);
    }

    /*
     * Arbitrary output speed calculation - we'll assume we instantly attain
     * velocity proportional to input voltage, up to a maximum of 100 rps
     * (this could be replaced with a real simulation if you want)
     */
    private double calculateSpeed(double currentVoltage) {
        return 100.0 * (currentVoltage / Util.MAX_VOLTS);
    }
}
