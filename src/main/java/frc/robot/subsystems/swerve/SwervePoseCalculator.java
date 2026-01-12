package frc.robot.subsystems.swerve;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import frc.robot.util.Util;

import java.util.ArrayList;
import java.util.List;
import java.util.Objects;
import java.util.function.Consumer;
import java.util.function.Supplier;

/**
 * <p>Implements the logic for "fusing" both odometry and vision-based pose
 * estimation. Also allows you to reset the current pose. See
 * the <a href="https://docs.wpilib.org/en/stable/docs/software/advanced-controls/state-space/state-space-pose-estimators.html">WPILib
 * docs</a> for more background on fused estimates.</p>
 *
 * <p>Also implements the ability to have "pose reset listeners". This is
 * for eventually supporting stuff like QuestNav, where it's important for
 * multiple systems to be aware of a pose reset.</p>
 *
 * <p>This class is implemented so it doesn't depend on a specific swerve
 * drive implementation or configuration; if it proves useful we might
 * later move it into a library.</p>
 */
public class SwervePoseCalculator {

    /**
     * Represents the different types of pose being managed
     */
    public enum PoseType {

        /** Odometry-only pose */
        ODOMETRY,

        /** Vision-only pose */
        VISION,

        /** Fused odometry + vision pose */
        FUSED
    }

    final Supplier<Rotation2d> headingGetter;
    final Supplier<SwerveModulePosition[]> modulePositionGetter;
    final SwerveDriveOdometry odometry;
    final SwerveDrivePoseEstimator estimator;
    final List<Consumer<Pose2d>> resetListeners;
    Pose2d latestVisionPose;
    Pose2d latestOdometryPose;
    Pose2d latestFusedPose;
    double latestVisionTimestamp;

    /**
     * Creates a {@link SwervePoseCalculator}
     * @param kinematics the chassis kinematics (required)
     * @param headingGetter a getter for the robot heading (required)
     * @param modulePositionGetter a getter for module positions (required)
     * @param initialPose the initial pose of the robot
     * @throws IllegalArgumentException if required parameters are null
     */
    public SwervePoseCalculator(SwerveDriveKinematics kinematics,
                                Supplier<Rotation2d> headingGetter,
                                Supplier<SwerveModulePosition[]> modulePositionGetter,
                                Pose2d initialPose) {

        Objects.requireNonNull(kinematics);

        // if there is no initial pose, we'll assume it's zero
        if (initialPose == null) {
            initialPose = Pose2d.kZero;
        }

        this.headingGetter = Objects.requireNonNull(headingGetter);
        this.modulePositionGetter = Objects.requireNonNull(modulePositionGetter);
        this.odometry = new SwerveDriveOdometry(
                kinematics,
                headingGetter.get(),
                modulePositionGetter.get(),
                initialPose);
        this.estimator = new SwerveDrivePoseEstimator(
                kinematics,
                headingGetter.get(),
                modulePositionGetter.get(),
                initialPose);
        this.resetListeners = new ArrayList<>();
        this.latestVisionPose = Util.NAN_POSE;
        this.latestOdometryPose = initialPose;
        this.latestFusedPose = initialPose;
        this.latestVisionTimestamp = Double.NaN;

        // add listeners for pose resets that will update odometry-only and
        // fused pose estimates
        resetListeners.add(newPose -> {
            odometry.resetPosition(
                    headingGetter.get(),
                    modulePositionGetter.get(),
                    newPose);
        });
        resetListeners.add(newPose -> {
            estimator.resetPosition(
                    headingGetter.get(),
                    modulePositionGetter.get(),
                    newPose);
        });
    }

    /**
     * Returns the most recently-computed pose estimate of the specified
     * type (as calculated by {@link #updateLatestPoseEstimates(boolean)}
     * @param type a pose type
     * @return the latest pose estimate of the specified type (this will never
     * be null, but may have NaNs if it's the vision pose and no vision
     * estimates have been provided)
     */
    public Pose2d getLatestPoseEstimate(PoseType type) {
        return switch (type) {
            case ODOMETRY -> latestOdometryPose;
            case VISION -> latestVisionPose;
            case FUSED -> latestFusedPose;
        };
    }

    /**
     * Add a vision pose to the estimator. The parameters indicate how much
     * to "trust" the vision estimate, and approximately how old it is (in
     * seconds since the robot started up). See these methods for more
     * information about the parameters:
     * <ul>
     *     <li>{@link SwerveDrivePoseEstimator#setVisionMeasurementStdDevs(Matrix)}</li>
     *     <li>{@link SwerveDrivePoseEstimator#addVisionMeasurement(Pose2d,double)}</li>
     * </ul>
     * @param pose the pose from the vision system
     * @param timestamp the timestamp of the vision measurement in seconds
     * @param stdDevs standard deviations of the vision measurements
     */
    public void addVisionEstimate(Pose2d pose, double timestamp, Matrix<N3,N1> stdDevs) {
        if (pose == null) {
            latestVisionPose = Util.NAN_POSE;
            latestVisionTimestamp = Double.NaN;
        } else {
            estimator.setVisionMeasurementStdDevs(stdDevs);
            estimator.addVisionMeasurement(pose, timestamp);
            latestVisionPose = pose;
            latestVisionTimestamp = timestamp;
        }
    }

    /**
     * Adds a reset listener - these are dudes that get notified whenever
     * we reset our pose
     *
     * @param listener new listener
     */
    public void addResetListener(Consumer<Pose2d> listener) {
        resetListeners.add(listener);
    }

    /**
     * Reset the pose of the robot to the specified value, and notify all
     * waiting listeners of the reset
     *
     * @param newPose the new pose of the robot
     */
    public void resetPose(Pose2d newPose) {

        // update latest values (no need to update the vision estimate)
        latestOdometryPose = newPose;
        latestFusedPose = newPose;

        // notify anyone listening for updates that we've reset
        for (Consumer<Pose2d> listener : resetListeners) {
            listener.accept(latestFusedPose);
        }
    }

    /**
     * Recalculates the latest pose estimates. You should call
     * this from a <code>periodic</code> method so you have access to the most
     * recent estimates.
     * @param trustVisionHeading true if we want to use the heading from the
     *                           fused pose estimate ((we've found that
     *                           Limelight heading estimates can be jittery,
     *                           so we usually don't trust it, and use the
     *                           odometry heading instead)
     */
    public void updateLatestPoseEstimates(boolean trustVisionHeading) {

        Rotation2d latestHeading = headingGetter.get();
        SwerveModulePosition [] latestPositions = modulePositionGetter.get();

        // update the odometry and calculate its pose estimate
        odometry.update(latestHeading, latestPositions);
        latestOdometryPose = odometry.getPoseMeters();

        // update the fused estimator and calculate its pose estimate
        estimator.update(latestHeading, latestPositions);
        latestFusedPose = estimator.getEstimatedPosition();

        // it may not be a good idea to trust the vision-based heading
        // estimate (the gyro is usually pretty accurate), so this may
        // wind up "overriding" it with the gyro heading from odometry
        if (!trustVisionHeading) {
            latestFusedPose = new Pose2d(
                    latestFusedPose.getX(),
                    latestFusedPose.getY(),
                    latestOdometryPose.getRotation());
        }
    }
}
