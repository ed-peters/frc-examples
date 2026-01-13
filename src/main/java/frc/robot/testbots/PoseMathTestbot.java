package frc.robot.testbots;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.swerve.PoseTrapezoid;
import frc.robot.subsystems.swerve.PoseTrapezoid.SwerveState;
import frc.robot.util.Util;

import static frc.robot.Config.SwerveSubsystem.alignRotateAcceleration;
import static frc.robot.Config.SwerveSubsystem.alignTranslateAcceleration;
import static frc.robot.Config.SwerveSubsystem.maxAlignRotate;
import static frc.robot.Config.SwerveSubsystem.maxAlignTranslate;

/**
 * Implementation of {@link TimedRobot} that lets us test various pose
 * math operations
 */
public class PoseMathTestbot extends TimedRobot {

    final PoseTrapezoid trapezoid;
    final Timer timer;
    Pose2d startPose;
    Pose2d calculatedPose;
    Pose2d finalPose;
    boolean restart;

    public PoseMathTestbot() {

        trapezoid = new PoseTrapezoid(
                maxAlignTranslate,
                alignTranslateAcceleration,
                maxAlignRotate,
                alignRotateAcceleration);
        timer = new Timer();

        Transform2d transform = new Transform2d(
            new Translation2d(10.0, 0.0),
            Rotation2d.k180deg);

        startPose = new Pose2d(1.0, 1.0, Rotation2d.fromDegrees(15.0));
        finalPose = startPose.transformBy(transform);
        calculatedPose = startPose;

        restart = true;
    }

    @Override
    public void robotPeriodic() {

        if (restart) {

            trapezoid.calculate(startPose, finalPose);

            Util.log("restarting ...");
            Util.log(" |- start pose = %s", startPose);
            Util.log(" |- final pose = %s", startPose);
            Util.log(" |- distance = %.2f", trapezoid.getDistance());
            Util.log(" |- angle = %.2f", trapezoid.getAngle());
            Util.log(" \\- time = %.2f", trapezoid.totalTime());

            calculatedPose = startPose;
            timer.restart();
            restart = false;

        } else if (timer.hasElapsed(trapezoid.totalTime() + 3.0)) {

            Util.log("stopping at %s", calculatedPose);
            restart = true;

        } else {

            SwerveState state = trapezoid.sample(timer.get());
            Util.publishPose("AutoStartPose", startPose);
            Util.publishPose("AutoFinalPose", finalPose);
            Util.publishPose("AutoNextPose", state.getPose());

            calculatedPose = Util.incrementPose(calculatedPose, state.getSpeeds());
            Util.publishPose("AutoCalculatedPose", calculatedPose);
        }
    }
}
