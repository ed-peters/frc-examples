package frc.robot.subsystems.arm;

import edu.wpi.first.math.MathUtil;

import static frc.robot.Config.Arm.maxAngle;
import static frc.robot.Config.Arm.maxVelocity;
import static frc.robot.Config.Arm.minAngle;

/**
 * Represents the setpoint for an arm. This includes both a "next" position and
 * velocity, as well as a "goal". We use that during long-term motion
 * planning when moving between two points.
 */
public class ArmSetpoint {

    double goalAngle;
    double nextAngle;
    double nextVelocity;

    /**
     * Creates a setpoint
     */
    public ArmSetpoint() {
        this.goalAngle = Double.NaN;
        this.nextAngle = Double.NaN;
        this.nextVelocity = Double.NaN;
    }

    /**
     * @return do we have a goal angle?
     */
    public boolean hasGoal() {
        return Double.isFinite(goalAngle);
    }

    /**
     * @return current "goal" angle
     */
    public double getGoalAngle() {
        return goalAngle;
    }

    /**
     * @return current "next" angle
     */
    public double getNextAngle() {
        return nextAngle;
    }

    /**
     * @return current "next" velocity
     */
    public double getNextVelocity() {
        return nextVelocity;
    }

    /**
     * Directs the arm to a target position and velocity, with an "end goal"
     * in mind
     *
     * @param goalAngle the end goal angle
     * @param nextAngle the next target angle
     * @param nextVelocity the next target velocity
     */
    public void set(double goalAngle, double nextAngle, double nextVelocity) {

        double minA = minAngle.getAsDouble();
        double maxA = maxAngle.getAsDouble();
        double maxV = maxVelocity.getAsDouble();

        // clamping positions is relatively easy
        double clampedGoal = MathUtil.clamp(goalAngle, minA, maxA);
        double clampedNext = MathUtil.clamp(nextAngle, minA, maxA);

        // clamping velocity is based on position - we don't want our velocity
        // to carry us above the max or below the min
        double clampedVelocity;
        if (clampedNext == minA) {
            clampedVelocity = MathUtil.clamp(nextVelocity, 0.0, maxV);
        } else if (clampedNext == maxA) {
            clampedVelocity = MathUtil.clamp(nextVelocity, -maxV, 0.0);
        } else {
            clampedVelocity = MathUtil.clamp(nextVelocity, -maxV, maxV);
        }

        this.goalAngle = clampedGoal;
        this.nextAngle = clampedNext;
        this.nextVelocity = clampedVelocity;
    }

    /**
     * Directs the arm to hold still at the current position (goal == next
     * and velocity is 0)
     *
     * @param holdAngle the angle to hold at
     */
    public void hold(double holdAngle) {
        set(holdAngle, holdAngle, 0.0);
    }

    /**
     * Clears the setpoint
     */
    public void clear() {
        this.goalAngle = Double.NaN;
        this.nextAngle = Double.NaN;
        this.nextVelocity = Double.NaN;
    }
}
