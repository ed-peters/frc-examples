package frc.robot.testbots;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.swerve.SwerveDriveWrapper;
import frc.robot.commands.swerve.SwerveTeleopCommand;
import frc.robot.commands.vision.LimelightTranslateCommand;
import frc.robot.subsystems.vision.LimelightSubsystem;
import frc.robot.subsystems.swerve.SimSwerveDriveSubsystem;
import frc.robot.subsystems.swerve.SwerveTeleopSpeedSupplier;
import frc.robot.subsystems.vision.Limelight.LimelightTarget;
import frc.robot.subsystems.swerve.SimSwerveChassis;

import static frc.robot.Config.SwerveSubsystem.kinematics;
import static frc.robot.Config.LimelightSubsystem.feetInFrontOfTag;
import static frc.robot.Config.LimelightTranslateTuning.desiredArea;
import static frc.robot.Config.LimelightTranslateTuning.desiredOffset;

/**
 * Implementation of {@link TimedRobot} that shows the use of the simulated
 * swerve chassis and the use of swerve commands
 */
public class LimelightTestbot extends TimedRobot {

    SimSwerveDriveSubsystem drive;
    LimelightSubsystem limelight;
    CommandXboxController controller;

    public LimelightTestbot() {

        drive = new SimSwerveDriveSubsystem(new SimSwerveChassis(kinematics));
        limelight = new LimelightSubsystem(
                drive::getOdometryPose,
                drive::addVisionEstimate);
        controller = new CommandXboxController(0);

        SwerveDriveWrapper wrapper = drive.getWrapper("auto");

        // default behavior is teleop using the normal bindings
        drive.setDefaultCommand(new SwerveTeleopCommand(
                wrapper,
                SwerveTeleopSpeedSupplier.create(controller)));

        // a will rotate a little bit left (spam it to turn around fully)
        controller.a().onTrue(wrapper.rotateCommand(15.0));

        // b will drive to one of the blue reef tags if it's closed enough
        controller.x().onTrue(wrapper.faceAprilTagCommand(
                17,
                10.0,
                feetInFrontOfTag.getAsDouble()));

        // x will orient the robot to face the current in-view tag
        controller.b().onTrue(driveToFaceCurrentTag(wrapper));

        // y will translate to the current in-view tag
        controller.y().onTrue(new LimelightTranslateCommand(
                wrapper,
                limelight::getTarget,
                desiredArea,
                desiredOffset));
    }

    /**
     * This shows how you can orient yourself in front of the current
     * in-view tag, if it's close enough.
     */
    private Command driveToFaceCurrentTag(SwerveDriveWrapper wrapper) {
        return wrapper.getSubsystem().defer(() -> {
            LimelightTarget target = limelight.getTarget();
            if (target == null || !target.isAprilTag()) {
                return null;
            }
            return wrapper.faceAprilTagCommand(
                    target.tagId(),
                    10.0,
                    feetInFrontOfTag.getAsDouble());
        });
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
    }
}
