package frc.robot.subsystems.swerve;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.math.util.Units;
import frc.robot.util.Trapezoid;
import frc.robot.util.Util;

import java.util.Objects;
import java.util.function.DoubleSupplier;

/**
 * Uses two {@link Trapezoid} motion profiles to calculate a field-relative
 * motion between two poses:
 * <ul>
 *
 *     <li>One for rotation, that will turn the robot smoothly from the
 *     start heading to the final heading</li>
 *
 *     <li>One for translation, that will move the robot smoothly along the
 *     straight line from the start pose to the end pose</li>
 *
 * </ul>
 *
 * Translation and rotation constrains are specified via {@link DoubleSupplier}
 * instances, so they can be managed via e.g. preferences. Each time you
 * reset, the motion can include translation and/or rotation components. This
 * can be used to achieve a target position, or heading, or both.
 */
public class PoseTrapezoid {

    // we're going to hardcode these because they seem unlikely to need tuning
    static final double MIN_RADIANS = Units.degreesToRadians(3.0);
    static final double MIN_METERS = Units.inchesToMeters(3.0);

    /**
     * Represents the state of the robot at some point on the trajectory
     */
    public class SwerveState {

        Pose2d pose;
        ChassisSpeeds speeds;

        /**
         * Creates a new {@link SwerveState}
         * @param pose the expected pose of the robot
         * @param speeds the expected speed of the robot
         */
        public SwerveState(Pose2d pose, ChassisSpeeds speeds) {
            this.pose = pose;
            this.speeds = speeds;
        }

        public Pose2d getPose() {
            return pose;
        }

        public ChassisSpeeds getSpeeds() {
            return speeds;
        }
    }

    final Trapezoid translation;
    final Trapezoid rotation;
    final SwerveState nextState;
    SwerveState startState;
    SwerveState finalState;
    boolean isTranslating;
    boolean isRotating;
    Pose2d startPose;
    Pose2d finalPose;
    double meters;
    double cos;
    double sin;
    double radians;
    double totalTime;

    /**
     * Creates a {@link PoseTrapezoid}
     * @param translateMaxVelocity supplies max translation velocity in ft/sec (required)
     * @param translateMaxAcceleration supplies max translation acceleration in ft/sec squared (required)
     * @param rotateMaxVelocity supplies max rotational velocity in deg/sec (required)
     * @param rotateMaxAcceleration supplies max rotational velocity in deg/sec squared (required)
     * @throws IllegalArgumentException if required parameters are null
     */
    public PoseTrapezoid(DoubleSupplier translateMaxVelocity,
                         DoubleSupplier translateMaxAcceleration,
                         DoubleSupplier rotateMaxVelocity,
                         DoubleSupplier rotateMaxAcceleration) {

        Objects.requireNonNull(translateMaxVelocity);
        Objects.requireNonNull(translateMaxAcceleration);
        Objects.requireNonNull(rotateMaxVelocity);
        Objects.requireNonNull(rotateMaxAcceleration);

        // externally we do degrees and feet as units, but in here we're going to
        // compute things using radians and meters because those are the native
        // units of WPILib - it will save us some time on unit conversions
        this.translation = new Trapezoid(
            () -> Units.feetToMeters(translateMaxVelocity.getAsDouble()), 
            () -> Units.feetToMeters(translateMaxAcceleration.getAsDouble()));
        this.rotation = new Trapezoid(
                () -> Units.degreesToRadians(rotateMaxVelocity.getAsDouble()),
                () -> Units.degreesToRadians(rotateMaxAcceleration.getAsDouble()));
        this.nextState = new SwerveState(null, null);
        this.isRotating = false;
        this.isTranslating = false;
        this.startPose = Util.NAN_POSE;
        this.finalPose = Util.NAN_POSE;
        this.cos = Double.NaN;
        this.sin = Double.NaN;
        this.totalTime = Double.NaN;
    }

    /**
     * @return current start pose
     */
    public Pose2d getFinalPose() {
        return finalPose;
    }

    /**
     * @return current final pose
     */
    public Pose2d getStartPose() {
        return startPose;
    }

    /**
     * @return degrees of rotation
     */
    public double getAngle() {
        return Units.radiansToDegrees(radians);
    }

    /**
     * @return feet of translation
     */
    public double getDistance() {
        return Units.metersToFeet(meters);
    }

    /**
     * Resets the motion profile
     * @param startPose starting pose (required)
     * @param finalPose final pose (required)
     * @throws IllegalArgumentException if required parameters are null
     */
    public void calculate(Pose2d startPose, Pose2d finalPose) {

        Objects.requireNonNull(startPose);
        Objects.requireNonNull(finalPose);

        this.totalTime = 0.0;
        this.startPose = startPose;
        this.finalPose = finalPose;
        this.startState = new SwerveState(startPose, Util.ZERO_SPEED);
        this.finalState = new SwerveState(finalPose, Util.ZERO_SPEED);

        // we implement translation by calculating an offset from the start
        // pose which moves along a straight line towards the final pose. we
        // will ignore translations that are too small.
        meters = Util.metersBetween(startPose, finalPose);
        if (MathUtil.isNear(0.0, meters, MIN_METERS)) {
            isTranslating = false;
        } else {
            isTranslating = true;
            translation.calculate(0.0, meters);
            totalTime = Math.max(totalTime, translation.totalTime());
        }

        // if we're translating, we need to know the angle of the line between
        // the start and final poses; the cos and sin of this angle will let us
        // decompose straight-line movement into separate X and Y components
        if (isTranslating) {
            Rotation2d translationAngle = finalPose.getTranslation()
                    .minus(startPose.getTranslation())
                    .getAngle();
            cos = translationAngle.getCos();
            sin = translationAngle.getSin();
        } else {
            cos = 0.0;
            sin = 0.0;
        }

        // we implement rotation by calculating an angle offset from the start
        // heading which moves towards the final heading. we ignore motions that
        // are too small.
        radians = Util.radiansBetween(startPose, finalPose);
        if (MathUtil.isNear(0.0, radians, MIN_RADIANS)) {
            isRotating = false;
        } else {
            isRotating = true;
            rotation.calculate(0.0, radians);
            totalTime = Math.max(totalTime, rotation.totalTime());
        }
    }

    /**
     * @return how long in seconds this motion will run for (this is the
     * maximum of the translation time and the rotation time)
     */
    public double totalTime() {
        return totalTime;
    }

    /**
     * @param t a time in seconds
     * @return is the motion finished at that time?
     */
    public boolean isFinishedAt(double t) {
        return t < totalTime();
    }

    /**
     * Calculates field-relative motion that will bring the robot from the
     * start pose to the final pose.
     *
     * @param t a time in seconds
     * @return the state of the motion at that time (for t&lt;0 this is the
     * start pose, for t&gt;totalTime this is the final pose)
     */
    public SwerveState sample(double t) {

        // if we're before the beginning or after the end, we will use either
        // the start or final pose and assume 0 speed
        if (t < 0) {
            return startState;
        } else if (t > totalTime) {
            return finalState;
        }

        double speedX = 0.0;
        double speedY = 0.0;
        double speedOmega = 0.0;
        double poseX = startPose.getX();
        double poseY = startPose.getY();
        double poseOmega = startPose.getRotation().getRadians();

        // if we're rotating, we get the state of the rotation profile. the
        // position will be the offset from the start heading at this moment
        // in time.
        if (isRotating) {
            State state = rotation.sample(t);
            poseOmega = MathUtil.angleModulus(poseOmega + state.position);
            speedOmega = state.velocity;
        }

        // if we're translating, we get the state of the translation profile.
        // the position and velocity will along the line from start to finish;
        // we decompose them along X/Y using cos/sin.
        if (isTranslating) {
            State state = translation.sample(t);
            speedX = state.velocity * cos;
            speedY = state.velocity * sin;
            poseX += state.position * cos;
            poseY += state.position * sin;
        }

        nextState.pose = new Pose2d(poseX, poseY, Rotation2d.fromRadians(poseOmega));
        nextState.speeds = new ChassisSpeeds(speedX, speedY, speedOmega);
        return nextState;
    }
}
