package frc.robot.commands.swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.util.Util;

import java.util.Objects;
import java.util.function.Supplier;

import static frc.robot.Config.SwerveTeleop.driverRelative;

/**
 * <p>This implements teleop driving, including the ability to use "driver
 * relative" speeds (so that pushing the joystick away from the driver always
 * sends the robot away from the driver, regardless of where the robot is or
 * what alliance you're on (see
 * {@link Util#fromDriverRelativeSpeeds(ChassisSpeeds, Rotation2d)}).</p>
 *
 * <p>This class is implemented using the {@link SwerveDriveWrapper} so it
 * doesn't depend on a specific swerve drive implementation; if it proves
 * useful, and we can figure out a better way to do configuration, we might
 * later move it into a library.</p>
 */
public class SwerveTeleopCommand extends Command {

    /**
     * Setting this to true will make this command publish a bunch of info
     * to the dashboard that might be helpful for debugging
     */
    static final boolean enableLogging = false;

    final SwerveDriveWrapper drive;
    final Supplier<ChassisSpeeds> desiredSpeedSupplier;
    ChassisSpeeds lastSpeed;

    /**
     * Creates a {@link SwerveTeleopCommand}
     * @param drive the swerve drive
     * @param desiredSpeedSupplier supplier for speed input (required)
     * @throws IllegalArgumentException if required parameters are null
     */
    public SwerveTeleopCommand(SwerveDriveWrapper drive,
                               Supplier<ChassisSpeeds> desiredSpeedSupplier) {

        this.drive = Objects.requireNonNull(drive);
        this.desiredSpeedSupplier = Objects.requireNonNull(desiredSpeedSupplier);
        this.lastSpeed = Util.NAN_SPEED;

        addRequirements(drive.getSubsystem());

        if (enableLogging) {
            SmartDashboard.putData("SwerveTeleopCommand", builder -> {
                builder.addDoubleProperty("LastX", () -> Units.metersToFeet(lastSpeed.vxMetersPerSecond), null);
                builder.addDoubleProperty("LastY", () -> Units.metersToFeet(lastSpeed.vyMetersPerSecond), null);
                builder.addDoubleProperty("LastOmega", () -> Units.radiansToDegrees(lastSpeed.omegaRadiansPerSecond), null);
                builder.addBooleanProperty("Running?", this::isScheduled, null);
            });
        }
    }

    @Override
    public void execute() {
        lastSpeed = desiredSpeedSupplier.get();
        if (driverRelative.getAsBoolean()) {
            lastSpeed = Util.fromDriverRelativeSpeeds(lastSpeed, drive.getHeading());
        }
        drive.driveRobotRelative(lastSpeed);
    }

    @Override
    public void end(boolean interrupted) {
        lastSpeed = Util.NAN_SPEED;
    }
}
