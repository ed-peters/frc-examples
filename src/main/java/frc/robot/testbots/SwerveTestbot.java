package frc.robot.testbots;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.swerve.SwerveDriveWrapper;
import frc.robot.subsystems.swerve.SimSwerveDriveSubsystem;
import frc.robot.commands.swerve.SwerveTeleopCommand;
import frc.robot.subsystems.swerve.SwerveTeleopSpeedSupplier;
import frc.robot.subsystems.swerve.SimSwerveChassis;
import frc.robot.util.ArenaWall;

import static frc.robot.Config.SwerveSubsystem.kinematics;

/**
 * Implementation of {@link TimedRobot} that shows the use of the simulated
 * swerve chassis and the use of swerve commands
 */
public class SwerveTestbot extends TimedRobot {

    SimSwerveDriveSubsystem drive;
    CommandXboxController controller;

    public SwerveTestbot() {

        drive = new SimSwerveDriveSubsystem(new SimSwerveChassis(kinematics));
        controller = new CommandXboxController(0);

        SwerveDriveWrapper wrapper = drive.getWrapper("auto");

        // default behavior is teleop using the normal bindings
        drive.setDefaultCommand(new SwerveTeleopCommand(
                wrapper,
                SwerveTeleopSpeedSupplier.create(controller)));

        // this maps the d-pad to let you point to specific arena walls
        controller.povUp().onTrue(wrapper.faceWallCommand(ArenaWall.NORTH));
        controller.povLeft().onTrue(wrapper.faceWallCommand(ArenaWall.WEST));
        controller.povDown().onTrue(wrapper.faceWallCommand(ArenaWall.SOUTH));
        controller.povRight().onTrue(wrapper.faceWallCommand(ArenaWall.EAST));

        // a will rotate a little bit left (spam it to turn around fully)
        controller.a().onTrue(wrapper.rotateCommand(15.0));

        // b will move the robot one meter straight ahead, and turn it around
        // to face the position it was just occupying
        controller.b().onTrue(wrapper.relativePoseCommand(new Pose2d(10.0, 0.0, Rotation2d.k180deg)));

        // x will rotate us slowly around the center of the blue reef
        controller.x().onTrue(rotateAroundReefCommand(wrapper, 90.0));

        // y will reset us to the zero pose
        controller.y().onTrue(drive.runOnce(() -> drive.resetPose(Pose2d.kZero)));

    }

    /**
     * This shows how you can combine (a) calculating a new heading so you are
     * facing an object on the field, and (b) using a custom center of rotation,
     * to rotate around a field object (in this case, the blue reef)
     */
    private Command rotateAroundReefCommand(SwerveDriveWrapper wrapper,
                                            double degreesPerSecond) {

        // this is the pose of the blue reef on the 2025 Reefscape fields
        // (we don't really care about the heading)
        final Pose2d reefCenter = new Pose2d(new Translation2d(
                Units.inchesToMeters(144.0 + (209.49 - 144.0) / 2.0),
                Units.inchesToMeters(130.17 + (186.82 - 130.17) / 2.0)),
                Rotation2d.kZero);

        // this is how fast we will rotate around the reef, once we're
        // facing it
        ChassisSpeeds rotateAroundReefSpeed = new ChassisSpeeds(
                0.0,
                0.0,
                Math.toRadians(degreesPerSecond));

        // we defer our calculations until the moment the command is run, since
        // they depend on the current position of the robot
        return drive.defer(() -> {

            // this command will point the robot towards the center of the reef
            Command faceReef = wrapper.pointAtCommand(reefCenter);

            // once we're facing the reef, the center of rotation is directly
            // in front of us, at the same distance it was earlier
            Translation2d centerOfRotation = new Translation2d(
                    reefCenter.relativeTo(drive.getPose()).getTranslation().getNorm(),
                    0.0);

            // this will move us around that center of rotation
            Command rotateAroundReef = drive.run(() -> {
                drive.drive(
                        "rotate-reef",
                        rotateAroundReefSpeed,
                        centerOfRotation);
            });

            return faceReef.andThen(rotateAroundReef);
        });
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
    }
}
