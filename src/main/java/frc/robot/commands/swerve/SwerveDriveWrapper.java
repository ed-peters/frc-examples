package frc.robot.commands.swerve;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.util.ArenaWall;
import frc.robot.util.Util;

import java.util.Objects;
import java.util.function.Consumer;
import java.util.function.Function;
import java.util.function.Supplier;

/**
 * <p>This lets us build fully-functional commands off an abstraction of a
 * swerve subsystem. Instances wrap a {@link Subsystem} and required functions
 * for driving around. Static methods act as factories for the various commands
 * in this package, and show how to do different kinds of pose-related math.</p>
 *
 * <p>The whole point of this class is not to depend on a specific swerve drive
 * implementation; if this proves useful, and we can figure out a better way to
 * do configuration, we might later move it into a library.</p>
 */
public class SwerveDriveWrapper {

//region Implementation --------------------------------------------------------

    final Subsystem subsystem;
    final Supplier<Pose2d> poseSupplier;
    final Consumer<Pose2d> poseConsumer;
    final Consumer<ChassisSpeeds> robotRelativeSpeedConsumer;

    /**
     * Creates a {@link SwerveDriveWrapper}
     * @param subsystem drive subsystem (required)
     * @param poseSupplier pose supplier (required)
     * @param poseResetConsumer consumer for pose reset requests (required)
     * @param robotRelativeSpeedConsumer consumer for robot-relative speeds (required)
     * @throws IllegalArgumentException if required parameters are null
     */
    public SwerveDriveWrapper(Subsystem subsystem,
                              Supplier<Pose2d> poseSupplier,
                              Consumer<Pose2d> poseResetConsumer,
                              Consumer<ChassisSpeeds> robotRelativeSpeedConsumer) {
        this.subsystem = Objects.requireNonNull(subsystem);
        this.poseSupplier = Objects.requireNonNull(poseSupplier);
        this.poseConsumer = Objects.requireNonNull(poseResetConsumer);
        this.robotRelativeSpeedConsumer = Objects.requireNonNull(robotRelativeSpeedConsumer);
    }

    /** @return the drive subsystem */
    public Subsystem getSubsystem() {
        return subsystem;
    }

    /** @return current pose */
    public Pose2d getPose() {
        return poseSupplier.get();
    }

    /** @return current heading */
    public Rotation2d getHeading() {
        return getPose().getRotation();
    }

    /** Resets the current pose */
    public void setPose(Pose2d pose) {
        poseConsumer.accept(pose);
    }

    /**
     * Drives the robot using robot-relative speeds
     * @param speeds desired robot speeds
     */
    public void driveRobotRelative(ChassisSpeeds speeds) {
        robotRelativeSpeedConsumer.accept(speeds);
    }

//endregion

//region Command factories -----------------------------------------------------

    /**
     * Creates a {@link SwerveAutoPoseCommand} that drives to the same
     * absolute position on the field each time it's run
     * @param finalPose final pose on the field (required)
     * @return the command
     * @throws IllegalArgumentException if required parameters are null
     */
    public Command absolutePoseCommand(Pose2d finalPose) {
        Objects.requireNonNull(finalPose);
        return new SwerveAutoPoseCommand(this, currentPose -> finalPose);
    }

    /**
     * Creates a {@link SwerveAutoPoseCommand} that drives to a relative
     * offset from the current position every time it's run
     * @param relativePose target pose, relative to the robot's pose when
     *                     the command starts (for instance, an X value of +1
     *                     would slide the robot 1m to the left when the
     *                     command runs) (required)
     * @return the command
     * @throws IllegalArgumentException if required parameters are null
     */
    public Command relativePoseCommand(Pose2d relativePose) {
        Objects.requireNonNull(relativePose);
        Transform2d transform = new Transform2d(
                relativePose.getTranslation(),
                relativePose.getRotation());
        return new SwerveAutoPoseCommand(
                this,
                currentPose -> currentPose.transformBy(transform));
    }

    /**
     * Creates a {@link SwerveAutoPoseCommand} that translates the robot to
     * an offset from the current position
     * @param x X offset in feet
     * @param y Y offset in feet
     * @return the command
     */
    public Command offsetCommand(double x, double y) {
        return relativePoseCommand(new Pose2d(
                Units.feetToMeters(x),
                Units.feetToMeters(y),
                Rotation2d.kZero));
    }

    /**
     * Creates a {@link SwerveAutoPoseCommand} that rotates the robot by
     * a fixed number of degrees
     * @param degrees the offset from the current position (required)
     * @return the command
     */
    public Command rotateCommand(double degrees) {
        return relativePoseCommand(new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(degrees)));
    }

    /**
     * Creates a {@link SwerveAutoPoseCommand} that rotates the robot to face
     * one of the arena walls
     * @param wall the arena wall to face (required)
     * @return the command
     * @throws IllegalArgumentException if required parameters are null
     */
    public Command faceWallCommand(ArenaWall wall) {
        return relativePoseCommand(new Pose2d(0.0, 0.0, wall.getFacingHeading()));
    }

    /**
     * Creates a {@link SwerveAutoPoseCommand} that points the robot at the
     * supplied point every time it's run
     * @param target target pose (required)
     * @return the command
     * @throws IllegalArgumentException if required parameters are null
     */
    public Command pointAtCommand(Pose2d target) {

        Objects.requireNonNull(target);

        Function<Pose2d,Pose2d> poseFunction = currentPose -> {

            // this calculates where the target pose is relative to the robot
            // at the moment the command starts
            Pose2d relativeTarget = target.relativeTo(currentPose);

            // this is the heading that will face the robot in the direction
            // of the target pose
            Rotation2d rotationAngle = currentPose.getRotation().plus(
                    relativeTarget.getTranslation().getAngle());

            // we will stay at the current pose but point in the required
            // direction
            return new Pose2d(
                    currentPose.getTranslation(),
                    rotationAngle);
        };

        return new SwerveAutoPoseCommand(this, poseFunction);
    }

    /**
     * This shows how, if you have a specific tag in mind, you can just tell
     * the robot to drive there. You don't want to drive automatically over
     * long distances, though - there's too much of a risk of obstacles. So
     * this will only work in a radius around the tag.
     */
    public Command faceAprilTagCommand(int tagId,
                                       double distanceLimit,
                                       double distanceOffset) {

        Function<Pose2d,Pose2d> poseFunction = currentPose -> {

            // get the pose facing the specified tag; if it's an invalid tag
            // we will do nothing
            Pose2d targetPose = Util.getPoseFacingTag(tagId, distanceOffset);
            if (targetPose == null) {
                return null;
            }

            // if that pose is too far away from where we are right now,
            // we will do nothing
            if (Util.feetBetween(currentPose, targetPose) > distanceLimit) {
                return null;
            }

            return targetPose;
        };

        return new SwerveAutoPoseCommand(this, poseFunction);
    }

//endregion

}
