package frc.robot.commands.vision;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Config.LimelightTranslateTuning;
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
import static frc.robot.Config.LimelightTranslateTuning.areaD;
import static frc.robot.Config.LimelightTranslateTuning.areaP;
import static frc.robot.Config.LimelightTranslateTuning.areaTolerance;
import static frc.robot.Config.LimelightTranslateTuning.offsetD;
import static frc.robot.Config.LimelightTranslateTuning.offsetP;
import static frc.robot.Config.LimelightTranslateTuning.offsetTolerance;

/**
 * <p>This uses the Limelight's X offset and pixel size data to center the
 * robot in front of an AprilTag. It moves the robot left/right and forward/back
 * to get to a reliable position in front of a tag, which is an important step in
 * targeting.</p>
 *
 * <p>This command assumes that the camera is facing out the front of the robot
 * It also assumes that the robot is facing the surface the tag is mounted on.
 * This is important so that, for example, +Y motion will move the camera
 * image to the left and won't also make the target smaller or bigger.</p>
 *
 * <p>Also note that this doesn't pick a specific tag - instead, it will use
 * whichever tag is currently in view, and will quit if the tag falls out of
 * view. If you want to ensure you're focusing on a specific tag, consider
 * using e.g. {@link Command#onlyWhile(BooleanSupplier)} as a guard.</p>
 *
 * <p>This class is implemented using the {@link SwerveDriveWrapper} so it
 * doesn't depend on a specific swerve drive implementation.</p>
 */
public class LimelightTranslateCommand extends Command {

    /**
     * Setting this to true will make this command publish a bunch of info
     * to the dashboard that might be helpful for debugging
     */
    static final boolean enableLogging = false;

    final SwerveDriveWrapper drive;
    final Supplier<LimelightTarget> targetSupplier;
    final PDController pidArea;
    final PDController pidOffset;
    final DoubleSupplier desiredArea;
    final DoubleSupplier desiredOffset;
    boolean achievedArea;
    boolean achievedOffset;
    double lastOffset;
    double lastArea;
    double lastSpeedX;
    double lastSpeedY;

    /**
     * Creates a {@link LimelightTranslateTuning}
     * @param drive the swerve drive (required)
     * @param targetSupplier supplies the current limelight target (required)
     * @param desiredArea supplies the desired area of the target when finished
     * @param desiredOffset supplies the desired offset of the target when finished
     * @throws IllegalArgumentException if required parameters are null
     */
    public LimelightTranslateCommand(SwerveDriveWrapper drive,
                                     Supplier<LimelightTarget> targetSupplier,
                                     DoubleSupplier desiredArea,
                                     DoubleSupplier desiredOffset) {
        this.drive = Objects.requireNonNull(drive);
        this.targetSupplier = Objects.requireNonNull(targetSupplier);
        this.desiredArea = desiredArea;
        this.desiredOffset = desiredOffset;
        this.pidArea = new PDController(areaP, areaD, areaTolerance);
        this.pidOffset = new PDController(offsetP, offsetD, offsetTolerance);
        addRequirements(drive.getSubsystem());
    }

    @Override
    public void initialize() {

        // if there's no tag in view, there's nothing to do
        LimelightTarget target = targetSupplier.get();
        if (target == null || !target.isAprilTag()) {
            Util.log("[swerve-servo] no tag in view; nothing to do");
            achievedArea = true;
            achievedOffset = true;
            return;
        }

        achievedArea = false;
        achievedOffset = false;

        // always reset the PIDs when you're doing closed loop
        pidArea.reset();
        pidOffset.reset();

        Util.log("[swerve-servo] aligning to tag %d", target.tagId());
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
            Util.log("[swerve-servo] TAG FELL OUT OF VIEW !!!");
            achievedOffset = true;
            achievedArea = true;
            return;
        }

        // area is how big the tag is in the camera frame; bigger means we're
        // closer to the tag (and hence we want to move in the -X direction)
        lastArea = target.area();

        // offset is how far to the right the tag is in the camera frame;
        // bigger means we're offset to the left (and want to move in the -Y
        // direction)
        lastOffset = target.offset();

        // calculate X speed (forward-back) if the tag is either too big or
        // to small in the camera frame
        lastSpeedX = 0.0;
        if (!achievedArea) {
            lastSpeedX = pidArea.calculate(lastArea, desiredArea.getAsDouble());
            achievedArea = pidArea.atSetpoint();
        }

        // calculate the Y speed (left-right) if the tag is to the right or
        // left of the desired tx
        lastSpeedY = 0.0;
        if (!achievedOffset) {
            lastSpeedY = pidOffset.calculate(lastOffset, desiredOffset.getAsDouble());
            achievedOffset = pidOffset.atSetpoint();
        }

        // calculate and clamp desired overall speeds
        ChassisSpeeds desiredSpeed = new ChassisSpeeds(
                lastSpeedX,
                lastSpeedY,
                0.0);
        Util.clampFeetAndDegrees(
                desiredSpeed,
                maxAlignTranslate,
                maxAlignRotate);

        drive.driveRobotRelative(desiredSpeed);

        // in normal operation, we're probably going to wind up with
        // many instances of this command. instead of trying to register
        // them all under different names, we'll just have whichever one
        // is running publish the "latest" information for debugging
        if (enableLogging) {
            SmartDashboard.putNumber("VisualServoCommand/SpeedX", lastSpeedX);
            SmartDashboard.putNumber("VisualServoCommand/SpeedY", lastSpeedY);
            SmartDashboard.putNumber("VisualServoCommand/OffsetCurrent", lastOffset);
            SmartDashboard.putNumber("VisualServoCommand/OffsetError", pidOffset.getError());
            SmartDashboard.putNumber("VisualServoCommand/AreaCurrent", lastArea);
            SmartDashboard.putNumber("VisualServoCommand/AreaError", pidArea.getError());
            SmartDashboard.putBoolean("VisualServoCommand/AtX?", pidOffset.atSetpoint());
            SmartDashboard.putBoolean("VisualServoCommand/AtY?", pidArea.atSetpoint());
            SmartDashboard.putBoolean("VisualServoCommand/Running?", true);
        }
    }

    @Override
    public boolean isFinished() {
        return achievedArea && achievedOffset;
    }

    @Override
    public void end(boolean interrupted) {
        lastArea = Double.NaN;
        lastOffset = Double.NaN;
        lastSpeedX = Double.NaN;
        lastSpeedY = Double.NaN;
        achievedArea = true;
        achievedOffset = true;
        if (enableLogging) {
            SmartDashboard.putBoolean("VisualServoCommand/Running?", false);
        }
    }
}
