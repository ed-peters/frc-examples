package frc.robot.subsystems.swerve;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.swerve.SwerveDriveWrapper;
import frc.robot.util.Util;
import frc.robot.subsystems.swerve.SwervePoseCalculator.PoseType;

import java.util.ArrayList;
import java.util.List;
import java.util.Objects;
import java.util.function.Consumer;

import static frc.robot.Config.SwerveSubsystem.cosineCompensation;
import static frc.robot.Config.SwerveSubsystem.kinematics;
import static frc.robot.Config.SwerveSubsystem.useFusedHeading;
import static frc.robot.Config.SwerveSubsystem.useFusedPose;

/**
 * <p>This is an implementation of a swerve drive subsystem that is meant to
 * wrap something that can handle the actual chassis commands (e.g. the
 * {@link SimSwerveChassis}. It was primarily written so we'd have an
 * actual subsystem to use for simulated testing.</p>
 *
 * <p>This class is implemented so it doesn't depend on a specific hardware
 * implementation; if it makes sense, and we can figure out a better way to
 * do configuration, we might later move it into a library.</p>
 */
public class SimSwerveDriveSubsystem extends SubsystemBase {

    final SwerveHardware hardware;
    final SwervePoseCalculator poseCalculator;
    final SwerveKinematicsCalculator kinematicsCalculator;
    final List<Consumer<Pose2d>> poseResetListeners;
    Rotation2d latestGyroHeading;
    Pose2d latestOdometryPose;
    Pose2d latestPoseEstimate;
    ChassisSpeeds latestSpeeds;
    String currentCommand;

    /**
     * Creates a {@link SimSwerveDriveSubsystem}
     * @param hardware the hardware for the drive (required)
     * @throws IllegalArgumentException if required parameters are null
     */
    public SimSwerveDriveSubsystem(SwerveHardware hardware) {

        this.hardware = Objects.requireNonNull(hardware);

        this.poseCalculator = new SwervePoseCalculator(
                kinematics,
                hardware::getHeading,
                hardware::getModulePositions,
                Pose2d.kZero);
        this.kinematicsCalculator = new SwerveKinematicsCalculator(
                kinematics,
                hardware::getModulePositions,
                cosineCompensation);
        this.poseResetListeners = new ArrayList<>();
        this.latestSpeeds = Util.ZERO_SPEED;
        this.latestGyroHeading = hardware.getHeading();
        this.latestOdometryPose = Pose2d.kZero;
        this.latestPoseEstimate = Pose2d.kZero;
        this.latestSpeeds = hardware.getCurrentSpeed();
        this.currentCommand = "";

        SmartDashboard.putData("SwerveDriveSubsystem", builder -> {
            builder.addStringProperty("CurrentCommand", () -> currentCommand, null);
            builder.addDoubleProperty("GyroHeading", () -> latestGyroHeading.getDegrees(), null);
            builder.addDoubleProperty("PoseX", () -> Units.metersToFeet(latestPoseEstimate.getX()), null);
            builder.addDoubleProperty("PoseY", () -> Units.metersToFeet(latestPoseEstimate.getY()), null);
            builder.addDoubleProperty("PoseDegrees", () -> latestPoseEstimate.getRotation().getDegrees(), null);
            builder.addDoubleProperty("SpeedX", () -> Units.metersToFeet(latestSpeeds.vxMetersPerSecond), null);
            builder.addDoubleProperty("SpeedY", () -> Units.metersToFeet(latestSpeeds.vyMetersPerSecond), null);
            builder.addDoubleProperty("SpeedOmega", () -> Units.radiansToDegrees(latestSpeeds.omegaRadiansPerSecond), null);
        });
    }

    /** @return kinematics for the drive */
    public SwerveDriveKinematics getKinematics() {
        return kinematics;
    }

    /** @return the heading of the robot */
    public Rotation2d getHeading() {
        return latestPoseEstimate.getRotation();
    }

    /**
     * @return the current pose estimate (which might be fused depending on
     * configuration)
     */
    public Pose2d getPose() {
        return latestPoseEstimate;
    }

    /**
     * @return the current pose based only on odometry
     */
    public Pose2d getOdometryPose() {
        return latestOdometryPose;
    }

    /** Supplies a vision estimate to the pose calculator */
    public void addVisionEstimate(Pose2d estimatedPose,
                                  double timestamp,
                                  Matrix<N3, N1> confidence) {
        poseCalculator.addVisionEstimate(estimatedPose, timestamp, confidence);
    }

    /**
     * Adds a listener that gets called whenever the robot's pose gets
     * reset (for instance, at the beginning of auto)
     */
    public void addPoseResetListener(Consumer<Pose2d> listener) {
        poseResetListeners.add(listener);
    }

    /**
     * Reset the pose of the robot to the specified value. This will also
     * notify anything listening for pose resets.
     */
    public void resetPose(Pose2d newPose) {
        poseCalculator.resetPose(newPose);
        for (Consumer<Pose2d> listener : poseResetListeners) {
            listener.accept(newPose);
        }
        latestPoseEstimate = newPose;
        Util.log("[swerve] reset pose to %s and notified %d listeners",
                newPose,
                poseResetListeners.size());
    }

    /**
     * Tells the robot to drive at the specified speeds in "robot
     * relative" coordinates, with the center of rotation being the
     * center of the robot
     */
    public void drive(String command, ChassisSpeeds speeds) {
        drive(command, speeds, Translation2d.kZero);
    }

    /**
     * Tells the robot to drive at the specified speeds in "robot
     * relative" coordinates, with the specified center of rotation
     */
    public void drive(String command, ChassisSpeeds speeds, Translation2d centerOfRotation) {
        currentCommand = command;
        latestSpeeds = speeds;
        hardware.setModuleStates(kinematicsCalculator.calculateStates(
                speeds,
                centerOfRotation));
    }

    /**
     * Updates and publishes odometry
     */
    @Override
    public void periodic() {

        // update pose information
        poseCalculator.updateLatestPoseEstimates(useFusedHeading.getAsBoolean());
        latestGyroHeading = hardware.getHeading();
        latestOdometryPose = poseCalculator.getLatestPoseEstimate(PoseType.ODOMETRY);

        Pose2d latestFusedPose = poseCalculator.getLatestPoseEstimate(PoseType.FUSED);
        latestPoseEstimate = useFusedPose.getAsBoolean()
                ? latestFusedPose
                : latestOdometryPose;

        // publish latest pose information
        Util.publishPose("FusedPose", latestFusedPose);
        Util.publishPose("OdometryPose", latestOdometryPose);
    }

    /**
     * @param commandName the command name to use when driving
     * @return a {@link SwerveDriveWrapper} pointing to this guy
     */
    public SwerveDriveWrapper getWrapper(String commandName) {
        Objects.requireNonNull(commandName);
        return new SwerveDriveWrapper(
                this,
                this::getPose,
                this::resetPose,
                speeds -> drive(commandName, speeds));
    }
}
