package frc.robot.subsystems.shooter;

import edu.wpi.first.math.controller.PIDController;
import frc.robot.util.Util;

/**
 * Implements {@link ShooterHardware} for simulation. This implementation is
 * a very idealized flywheel, with software PID control.
 */
public class ShooterSim implements ShooterHardware {

    final PIDController pid;
    boolean brake = false;
    double velocity = 0.0;
    double position = 0.0;
    double v = 0.0;

    public ShooterSim() {
        pid = new PIDController(0.0, 0.0, 0.0);
    }

    @Override
    public boolean isBrakeEnabled() {
        return brake;
    }

    @Override
    public void setBrake(boolean brake) {
        this.brake = brake;
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
            velocity = calculateCoasting(velocity);
        } else {
            velocity = calculateSpeed(volts);
        }
        position += velocity * Util.DT;
    }

    @Override
    public void resetPid(double p, double d, double v) {
        this.v = v;
        pid.setPID(p, 0.0, d);
        pid.reset();
    }

    @Override
    public void applySpeed(double rps) {
        double ff = v * rps;
        double fb = pid.calculate(getMotorSpeed(), rps);
        applyVolts(Util.clampVolts(ff + fb));
    }

    /*
     * Arbitrary "coasting" calculation. If we're braking, we assume we stop
     * immediately. Otherwise, we assume speed drops off by 1% every cycle
     * until it gets close to a minimum speed (this could be replaced with
     * a real simulation if you want).
     */
    private double calculateCoasting(double currentVelocity) {
        if (brake) {
            return 0.0;
        }
        return Math.min(2.0, currentVelocity * 0.99);
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
