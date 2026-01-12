package frc.robot.subsystems.vision;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.Util;
import frc.robot.subsystems.vision.Limelight.LimelightTarget;
import frc.robot.subsystems.vision.LimelightHelpers.PoseEstimate;

import java.util.Objects;
import java.util.function.Supplier;

import static frc.robot.Config.LimelightSubsystem.limelightName;

/**
 * Subsystem for managing the Limelight
 */
public class LimelightSubsystem extends SubsystemBase {

    /**
     * Confidence value to supply with the vision estimate when using the
     * MegaTag2 algorithm
     */
    public static final Vector<N3> confidence = VecBuilder.fill(.7,.7,9999999);

    /**
     * Interface for components that consume pose estimates
     */
    public interface VisionEstimateConsumer {

        void consume(Pose2d estimatedPose,
                     double timestamp,
                     Matrix<N3,N1> confidence);
    }

    final Supplier<Pose2d> poseSupplier;
    final VisionEstimateConsumer consumer;
    final Limelight limelight;
    LimelightSim sim;
    LimelightTarget target;
    PoseEstimate estimate;

    public LimelightSubsystem(Supplier<Pose2d> odometryPoseSupplier,
                              VisionEstimateConsumer consumer) {

        this.poseSupplier = Objects.requireNonNull(odometryPoseSupplier);
        this.consumer = Objects.requireNonNull(consumer);
        this.limelight = new Limelight(limelightName, () -> odometryPoseSupplier.get().getRotation());

        if (RobotBase.isSimulation()) {
            sim = new LimelightSim(limelightName);
        }

        SmartDashboard.putData("LimelightSubsystem", builder -> {
            builder.addBooleanProperty("HasEstimate?", () -> estimate != null, null);
            builder.addBooleanProperty("HasTarget?", this::hasTarget, null);
            builder.addDoubleProperty("TargetTagId", this::getVisibleTagId, null);
        });
    }

    /**
     * @return do we have a target in view?
     */
    public boolean hasTarget() {
        return target != null;
    }

    /**
     * @return the ID of the current tag in view, -1 if there isn't one
     */
    public int getVisibleTagId() {
        if (target == null || !target.isAprilTag()) {
            return 0;
        }
        return target.tagId();
    }

    /**
     * @return the pose of the current tag in view, null if there isn't one
     */
    public Pose2d getVisibleTagPose() {
        if (target == null || !target.isAprilTag()) {
            return null;
        }
        return Util.getTagPose(target.tagId());
    }

    /**
     * @return the current pose estimate, null if there isn't one
     */
    public Pose2d getPoseEstimate() {
        return estimate != null ? estimate.pose : null;
    }

    /**
     * @return the latest vision target (null if there isn't one)
     */
    public LimelightTarget getTarget() {
        return target;
    }

    @Override
    public void periodic() {

        // capture the latest vision estimate
        estimate = limelight.getPoseEstimate();
        if (estimate != null) {
            consumer.consume(
                    estimate.pose,
                    estimate.timestampSeconds,
                    confidence);
        }

        // capture the latest target; if there is one, and it's a tag,
        // publish its pose
        target = limelight.getTarget();
        if (target != null && target.isAprilTag()) {
            Pose2d tagPose = getVisibleTagPose();
            Util.publishPose("VisionTarget", tagPose == null
                    ? Util.NAN_POSE
                    : tagPose);
        }

        // always publish poses
        Util.publishPose("LimelightTarget", getVisibleTagPose());
        Util.publishPose("LimelightPose", getPoseEstimate());

        // update the simulation if we can
        if (sim != null) {
            sim.updateFakePoses(poseSupplier.get());
        }
    }
}
