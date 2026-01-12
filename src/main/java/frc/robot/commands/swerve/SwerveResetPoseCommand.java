package frc.robot.commands.swerve;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;

import java.util.Objects;
import java.util.function.Supplier;

import static frc.robot.util.Util.log;

/**
 * <p>Resets the current pose of the swerve drive from a {@link Supplier}.
 * This can be used to reset the pose of the swerve drive to the latest
 * vision estimate; this is insanely useful in testing on a practice field,
 * either at school or at a competition.</p>
 *
 * <p>This class is implemented using the {@link SwerveDriveWrapper} so it
 * doesn't depend on a specific swerve drive implementation; if it proves
 * useful, and we can figure out a better way to do configuration, we might
 * later move it into a library.</p>
 */
public class SwerveResetPoseCommand extends Command{

    final SwerveDriveWrapper drive;
    final Supplier<Pose2d> poseSupplier;

    /**
     * Creates a {@link SwerveResetPoseCommand}
     * @param drive the swerve drive
     * @param poseSupplier supplier for the new pose
     * @throws IllegalArgumentException if required parameters are null
     */
    public SwerveResetPoseCommand(SwerveDriveWrapper drive,
                                  Supplier<Pose2d> poseSupplier) {

        this.drive = Objects.requireNonNull(drive);
        this.poseSupplier = Objects.requireNonNull(poseSupplier);

        // unlike most commands, we won't actually depend on a subsystem;
        // we don't need to consume a full scheduler cycle to do this
    }

    @Override
    public void execute() {
        Pose2d pose = poseSupplier.get();
        if (pose != null) {
            drive.setPose(pose);
        } else {
            log("[swerve] can't reset pose - no new value available");
        }
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
