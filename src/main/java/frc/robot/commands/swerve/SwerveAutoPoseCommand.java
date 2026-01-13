package frc.robot.commands.swerve;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.util.PDController;
import frc.robot.subsystems.swerve.PoseTrapezoid;
import frc.robot.subsystems.swerve.PoseTrapezoid.SwerveState;
import frc.robot.util.Util;

import java.util.Objects;
import java.util.function.Function;

import static frc.robot.Config.SwerveSubsystem.maxAlignTranslate;
import static frc.robot.Config.SwerveSubsystem.maxAlignRotate;
import static frc.robot.Config.SwerveSubsystem.alignRotateAcceleration;
import static frc.robot.Config.SwerveSubsystem.alignTranslateAcceleration;
import static frc.robot.Config.SwerveAutoTuning.rotateD;
import static frc.robot.Config.SwerveAutoTuning.rotateP;
import static frc.robot.Config.SwerveAutoTuning.rotateTolerance;
import static frc.robot.Config.SwerveAutoTuning.translateD;
import static frc.robot.Config.SwerveAutoTuning.translateP;
import static frc.robot.Config.SwerveAutoTuning.translateTolerance;

/**
 * <p>This command drives the robot to a fixed pose on the field using a
 * {@link PoseTrapezoid}. It can be used to orient to a
 * specific heading, perform translations, or both.</p>
 *
 * <p>This class is implemented using the {@link SwerveDriveWrapper} so it
 * doesn't depend on a specific swerve drive implementation; if it proves
 * useful, and we can figure out a better way to do configuration, we might
 * later move it into a library.</p>
 */
public class SwerveAutoPoseCommand extends Command {

    public static final boolean DEBUG = false;

    final SwerveDriveWrapper drive;
    final PDController pidX;
    final PDController pidY;
    final PDController pidOmega;
    final Timer timer;
    final PoseTrapezoid trapezoid;
    Function<Pose2d,Pose2d> poseFunction;
    boolean initialized;

    /**
     * Creates a {@link SwerveAutoPoseCommand} that calculates its target
     * pose based on the robot's current pose when it starts executing
     *
     * @param drive the swerve drive
     * @param poseFunction calculates target pose based on current pose (required)
     * @throws IllegalArgumentException if required parameters are null
     */
    public SwerveAutoPoseCommand(SwerveDriveWrapper drive,
                                 Function<Pose2d,Pose2d> poseFunction) {

        this.drive = Objects.requireNonNull(drive);
        this.poseFunction = Objects.requireNonNull(poseFunction);
        this.pidX = new PDController(translateP, translateD, translateTolerance);
        this.pidY = new PDController(translateP, translateD, translateTolerance);
        this.pidOmega = new PDController(rotateP, rotateD, rotateTolerance);
        this.timer = new Timer();
        this.poseFunction = poseFunction;
        this.trapezoid = new PoseTrapezoid(
                maxAlignTranslate,
                alignTranslateAcceleration,
                maxAlignRotate,
                alignRotateAcceleration);
        this.initialized = false;

        // the turning PID is use for angle calculations in degrees, so we
        // want it to provide wraparound
        pidOmega.enableContinuousInput(-180.0, 180.0);

        addRequirements(drive.getSubsystem());
    }

    /**
     * Captures the start pose, reset the motion profile and start timing
     */
    @Override
    public void initialize() {

        // calculate the start/final poses; if the target pose is null,
        // we aren't going to do anything
        Pose2d startPose = drive.getPose();
        Pose2d finalPose = poseFunction.apply(startPose);
        if (finalPose == null) {
            Util.log("[swerve-auto] no final pose; I refuse to do anything");
            initialized = false;
            return;
        }

        // calculate the motion profile
        trapezoid.calculate(startPose, finalPose);

        if (DEBUG) {
            Util.log("[swerve-auto] We're on our way!");
            Util.log(" --> start pose = %s", startPose);
            Util.log(" --> final pose = %s", finalPose);
            Util.log(" --> distance = %.2f", trapezoid.getDistance());
            Util.log(" --> angle = %.2f", trapezoid.getAngle());
            Util.log(" --> time = %.2f", trapezoid.totalTime());
        }

        // reset the PID controllers for each direction of motion
        pidX.reset();
        pidY.reset();
        pidOmega.reset();

        // sneaky trick - our PID gains and tolerances are going to set based
        // on feet & degrees, but in execute() we're working with meters and
        // radians, so we convert them here
        pidX.setTolerance(Units.feetToMeters(pidX.getErrorTolerance()));
        pidY.setTolerance(Units.feetToMeters(pidY.getErrorTolerance()));
        pidOmega.setTolerance(Units.degreesToRadians(pidOmega.getErrorTolerance()));

        // let's roll ...
        timer.restart();
        initialized = true;
    }

    // ===============================================================
    // EXECUTION
    // ===============================================================

    @Override
    public void execute() {

        // if we're skipping, there's nothing to do
        if (!initialized) {
            return;
        }
        
        // first we calculate where we're supposed to be, based on the
        // calculated motion profile
        SwerveState desiredState = trapezoid.sample(timer.get());
        ChassisSpeeds desiredSpeeds = desiredState.getSpeeds();
        Pose2d desiredPose = desiredState.getPose();
        Pose2d currentPose = drive.getPose();

        // we tweak the desired speeds using PID to correct for 
        // position inaccuracy
        desiredSpeeds.vxMetersPerSecond += pidX.calculate(currentPose.getX(), desiredPose.getX());
        desiredSpeeds.vyMetersPerSecond += pidY.calculate(currentPose.getY(), desiredPose.getY());
        desiredSpeeds.omegaRadiansPerSecond += pidOmega.calculate(
            currentPose.getRotation().getDegrees(), 
            desiredPose.getRotation().getDegrees());

        // clamp the speeds to make sure we're not moving too fast
        Util.clampFeetAndDegrees(
                desiredSpeeds,
                maxAlignTranslate,
                maxAlignRotate);

        // since the PoseProfile gives us field-relative speeds, we need to
        // translate them into robot-relative speeds so we can tell the
        // robot to drive the intended path
        ChassisSpeeds robotRelativeSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                desiredSpeeds,
                currentPose.getRotation());

        SmartDashboard.putBoolean("SwerveAutoPoseCommand/Running?", true);
        SmartDashboard.putNumber("SwerveAutoPoseCommand/SpeedX", Units.metersToFeet(robotRelativeSpeeds.vxMetersPerSecond));
        SmartDashboard.putNumber("SwerveAutoPoseCommand/SpeedY", Units.metersToFeet(robotRelativeSpeeds.vyMetersPerSecond));
        SmartDashboard.putNumber("SwerveAutoPoseCommand/SpeedOmega", Math.toDegrees(robotRelativeSpeeds.omegaRadiansPerSecond));
        SmartDashboard.putNumber("SwerveAutoPoseCommand/ErrorX", pidX.getError());
        SmartDashboard.putNumber("SwerveAutoPoseCommand/ErrorY", pidY.getError());
        SmartDashboard.putNumber("SwerveAutoPoseCommand/ErrorOmega", pidOmega.getError());
        Util.publishPose("AutoPoseNext", desiredPose);
        Util.publishPose("AutoPoseFinal", trapezoid.getFinalPose());

        drive.driveRobotRelative(robotRelativeSpeeds);
    }

    @Override
    public boolean isFinished() {

        // since we direct motion using a timed profile, we base our decision
        // of "doneness" on time rather than position. that way, if we get
        // bumped or are mis-tuned, the command will quit instead of waiting
        // forever to get to a position it will never reach.
        return !initialized || timer.hasElapsed(trapezoid.totalTime());
    }

    @Override
    public void end(boolean interrupted) {
        initialized = false;
        SmartDashboard.putBoolean("SwerveAutoPoseCommand/Running?", false);
    }
}
