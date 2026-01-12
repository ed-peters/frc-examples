package frc.robot.commands.vision;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.swerve.SwerveDriveWrapper;
import frc.robot.util.Util;
import frc.robot.util.PDController;
import frc.robot.subsystems.vision.Limelight.LimelightTarget;

import java.util.Objects;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import static frc.robot.Config.SwerveSubsystem.maxAlignRotate;
import static frc.robot.Config.SwerveSubsystem.maxAlignTranslate;
import static frc.robot.Config.LimelightSubsystem.mountAngle;
import static frc.robot.Config.LimelightSubsystem.mountHeightAboveFloor;
import static frc.robot.Config.LimelightRotateTuning.distanceD;
import static frc.robot.Config.LimelightRotateTuning.distanceP;
import static frc.robot.Config.LimelightRotateTuning.distanceTolerance;
import static frc.robot.Config.LimelightRotateTuning.rotateD;
import static frc.robot.Config.LimelightRotateTuning.rotateP;
import static frc.robot.Config.LimelightRotateTuning.rotateTolerance;

/**
 * <p>This command uses the Limelight's horizontal offset (TX) and vertical
 * angle (TY) to position the robot at a desired distance from an AprilTag.
 * This is roughly what they suggest in their documentation as <a href="https://docs.limelightvision.io/docs/docs-limelight/tutorials/tutorial-aiming-with-visual-servoing">visul
 * servoing</a>. This command operates in two steps:</p>
 * <ol>
 *   <li>Rotates the robot until the target is centered (offset = 0)</li>
 *   <li>Translates forward/backward until the robot is at the desired distance</li>
 * </ol>
 *
 * <p>Distance is calculated using the camera mount height, mount angle, target
 * height, and the vertical angle (TY) to the target.</p>
 *
 * <p>This command assumes that the camera is facing out the front of the robot.
 * It also assumes that the robot is roughly facing the tag surface. Unlike
 * {@link LimelightTranslateCommand}, this command will handle the initial alignment.</p>
 *
 * <p>Also note that this doesn't pick a specific tag - instead, it will use
 * whichever tag is currently in view, and will quit if the tag falls out of
 * view. If you want to ensure you're focusing on a specific tag, consider
 * using e.g. {@link Command#onlyWhile(BooleanSupplier)} as a guard.</p>
 *
 * <p>This class is implemented using the {@link SwerveDriveWrapper} so it
 * doesn't depend on a specific swerve drive implementation.</p>
 */
public class LimelightRotateCommand extends Command {

    /**
     * Setting this to true will make this command publish a bunch of info
     * to the dashboard that might be helpful for debugging
     */
    static final boolean enableLogging = false;

    final SwerveDriveWrapper drive;
    final Supplier<LimelightTarget> targetSupplier;
    final PDController pidRotate;
    final PDController pidDistance;
    final DoubleSupplier cameraMountAngle;
    final DoubleSupplier cameraMountHeight;
    final DoubleSupplier targetHeightAboveFloor;
    final DoubleSupplier desiredDistance;
    boolean achievedRotation;
    boolean achievedDistance;
    double lastOffset;
    double lastDistance;
    double lastSpeedX;
    double lastSpeedRotate;

    /**
     * Creates a {@link LimelightRotateCommand}
     * @param drive the swerve drive (required)
     * @param targetSupplier supplies the current limelight target (required)
     * @param cameraMountHeight the height of the camera mount in inches
     * @param cameraMountAngle the angle of the camera mount in degrees
     * @param targetHeightAboveFloor the height of the target above the floor
     * @param desiredDistance the desired final distance from the target in feet
     */
    public LimelightRotateCommand(SwerveDriveWrapper drive,
                                  Supplier<LimelightTarget> targetSupplier,
                                  DoubleSupplier cameraMountHeight,
                                  DoubleSupplier cameraMountAngle,
                                  DoubleSupplier targetHeightAboveFloor,
                                  DoubleSupplier desiredDistance) {
        this.drive = Objects.requireNonNull(drive);
        this.targetSupplier = Objects.requireNonNull(targetSupplier);
        this.cameraMountHeight = cameraMountHeight;
        this.cameraMountAngle = cameraMountAngle;
        this.targetHeightAboveFloor = targetHeightAboveFloor;
        this.desiredDistance = desiredDistance;
        this.pidRotate = new PDController(rotateP, rotateD, rotateTolerance);
        this.pidDistance = new PDController(distanceP, distanceD, distanceTolerance);
        addRequirements(drive.getSubsystem());
    }

    @Override
    public void initialize() {

        // if there's no tag in view, there's nothing to do
        LimelightTarget target = targetSupplier.get();
        if (target == null || !target.isAprilTag()) {
            Util.log("[limelight-rotate] no tag in view; nothing to do");
            achievedRotation = true;
            achievedDistance = true;
            return;
        }

        achievedRotation = false;
        achievedDistance = false;

        // always reset the PIDs when you're doing closed loop
        pidRotate.reset();
        pidDistance.reset();

        Util.log("[limelight-rotate] aligning to tag %d at %.1f feet", target.tagId(), desiredDistance);
    }

    @Override
    public void execute() {

        // if we're not running, there's nothing to do
        if (isFinished()) {
            return;
        }

        // we can only do this if we have a tag in view; if our tag fell
        // out of view, we'll warn and then quit
        LimelightTarget target = targetSupplier.get();
        if (target == null) {
            Util.log("[limelight-rotate] TAG FELL OUT OF VIEW !!!");
            achievedRotation = true;
            achievedDistance = true;
            return;
        }

        // these are the speeds we will calculate
        lastSpeedRotate = 0.0;
        lastSpeedX = 0.0;

        // these are the values we use for targeting
        lastOffset = target.offset();
        lastDistance = calculateDistance(target.angle());

        // we will rotate first, and only worry about forward/back once
        // we've centered on the tag
        if (!achievedRotation) {
            lastSpeedRotate = Util.applyClamp(
                    pidRotate.calculate(lastOffset, 0.0),
                    maxAlignRotate);
            achievedRotation = pidRotate.atSetpoint();
        } else if (!achievedDistance) {
            lastSpeedX = Util.applyClamp(
                    pidDistance.calculate(lastDistance, desiredDistance.getAsDouble()),
                    maxAlignTranslate);
            achievedDistance = pidDistance.atSetpoint();
        }

        drive.driveRobotRelative(new ChassisSpeeds(
                lastSpeedX,
                0.0,
                Units.degreesToRadians(lastSpeedRotate)));

        // publish debugging info if enabled
        if (enableLogging) {
            SmartDashboard.putNumber("LimelightRotateCommand/SpeedX", lastSpeedX);
            SmartDashboard.putNumber("LimelightRotateCommand/SpeedRotate", lastSpeedRotate);
            SmartDashboard.putNumber("LimelightRotateCommand/OffsetCurrent", lastOffset);
            SmartDashboard.putNumber("LimelightRotateCommand/OffsetError", pidRotate.getError());
            SmartDashboard.putNumber("LimelightRotateCommand/DistanceCurrent", lastDistance);
            SmartDashboard.putNumber("LimelightRotateCommand/DistanceError", pidDistance.getError());
            SmartDashboard.putBoolean("LimelightRotateCommand/AtRotation?", pidRotate.atSetpoint());
            SmartDashboard.putBoolean("LimelightRotateCommand/AtDistance?", pidDistance.atSetpoint());
            SmartDashboard.putBoolean("LimelightRotateCommand/Running?", true);
        }
    }

    /**
     * Calculates the horizontal distance to the target using trigonometry.
     * The <a href="https://docs.limelightvision.io/docs/docs-limelight/tutorials/tutorial-estimating-distance">Limelight
     * docs</a> explain how this works in detail.
     *
     * @param verticalAngle the vertical angle from the camera to the target (TY)
     * @return the horizontal distance to the target in feet
     */
    private double calculateDistance(double verticalAngle) {

        // Total vertical angle from horizontal to target
        double totalAngle = mountAngle + verticalAngle;

        // Convert to radians for calculation
        double totalAngleRad = Units.degreesToRadians(totalAngle);

        // Calculate horizontal distance using trigonometry
        // distance = (targetHeight - cameraHeight) / tan(angle)
        double heightDifference = targetHeightAboveFloor.getAsDouble() - mountHeightAboveFloor;
        double distanceInches = heightDifference / Math.tan(totalAngleRad);

        // Convert inches to feet
        return distanceInches / 12.0;
    }

    @Override
    public boolean isFinished() {
        return achievedRotation && achievedDistance;
    }

    @Override
    public void end(boolean interrupted) {
        lastOffset = Double.NaN;
        lastDistance = Double.NaN;
        lastSpeedX = Double.NaN;
        lastSpeedRotate = Double.NaN;
        achievedRotation = true;
        achievedDistance = true;
        if (enableLogging) {
            SmartDashboard.putBoolean("LimelightRotateCommand/Running?", false);
        }
    }
}
