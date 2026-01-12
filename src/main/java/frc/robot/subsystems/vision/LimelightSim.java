package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.util.Util;

import java.util.Arrays;

/**
 * Simulates a Limelight camera. Use it by calling
 * {@link #updateFakePoses(Pose2d)} in a periodic method somewhere. It will:
 * <ul>
 *
 *     <li>Determine whether the robot can "see" any known AprilTags based
 *     on its supplied position on the field, and some configuration around
 *     e.g. the FOV of the camera</li>
 *
 *     <li>Based on the answer to the above question, update both pose estimate
 *     AND visual targeting (acting a as a "virtual Limelight").</li>
 *
 * </ul>
 *
 * This should be useful for testing targeting.
 */
public class LimelightSim {

    /**
     * Key in SmartDashboard for mega tag pose estimates
     */
    public static final String MEGA_TAG_KEY = "botpose_orb_wpiblue";

    /**
     * Placeholder for no tag
     */
    public static final double [] NO_TAG = new double[0];

    /**
     * This is the angle (in degrees) of the cone in front of the robot that
     * will be used for detecting tags
     */
    public static final double DEFAULT_DETECTION_ANGLE = 120.0;

    /**
     * This is the maximum distance away (in feet) a tag can be from the robot
     * for us to consider it visible
     */
    public static final double DEFAULT_DETECTION_DISTANCE = 6.0;

    /**
     * We add a little "noise" to the LL pose estimates, so we can tell them
     * apart from the simulated odometry position of the robot; this is how
     * far we tx the simulated position from the robot's actual pose
     */
    public static final double DEFAULT_POSE_ERROR = Units.inchesToMeters(4.0);

    final DoubleArrayPublisher posePublisher;
    final DoublePublisher tvPublisher;
    final DoublePublisher taPublisher;
    final DoublePublisher txPublisher;
    final DoublePublisher tidPublisher;
    double detectionDistance;
    double detectionAngle;
    double poseError;

    public LimelightSim(String limelightName) {

        this.detectionDistance = DEFAULT_DETECTION_DISTANCE;
        this.detectionAngle = DEFAULT_DETECTION_ANGLE;
        this.poseError = DEFAULT_POSE_ERROR;

        NetworkTable table = NetworkTableInstance.getDefault().getTable(limelightName);
        posePublisher = table.getDoubleArrayTopic(MEGA_TAG_KEY).publish();
        tidPublisher = table.getDoubleTopic("tid").publish();
        taPublisher = table.getDoubleTopic("ta").publish();
        txPublisher = table.getDoubleTopic("tx").publish();
        tvPublisher = table.getDoubleTopic("tv").publish();

        SmartDashboard.putData("LimelightSim", builder -> {
            builder.addDoubleProperty("DetectionAngle", () -> detectionAngle, val -> detectionAngle = val);
            builder.addDoubleProperty("DetectionRadius", () -> detectionDistance, val -> detectionDistance = val);
            builder.addDoubleProperty("PoseError", () -> poseError, val -> poseError = val);
        });
    }

    /**
     * Called every frame to determine whether we can see a tag, and supply
     * targeting data for it
     *
     * @param odometryPose the pose of the robot as determined ONLY by
     *                     odometry (if it includes a vision estimate, we will
     *                     compound error until things go really badly)
     */
    public void updateFakePoses(Pose2d odometryPose) {

        AprilTag closestTagInView = Util.closestTagFilteredBy(odometryPose, thisTag -> {

            Pose2d thisPose = thisTag.pose.toPose2d();

            // basic distance check - if a tag is beyond our detection
            // distance, we'll ignore it
            double thisDistance = Util.feetBetween(odometryPose, thisPose);
            if (thisDistance > detectionDistance) {
                return false;
            }

            // this gives us the tag's position relative to the robot; +X
            // means the tag is in front of the robot; +Y means it's to the
            // left of the robot
            Translation2d thisTranslation = thisPose
                    .relativeTo(odometryPose)
                    .getTranslation();

            // camera angle check - we get the angle of the tag off the robot's
            // straight-line heading; this is the "field of view". if it's too
            // big, we will ignore the tag
            double halfAngle = detectionAngle / 2.0;
            double degrees = thisTranslation.getAngle().getDegrees();
            if (degrees > halfAngle && degrees < -halfAngle) {
                return false;
            }

            // if we passed all these tests, we consider this tag to be "in view"
            return true;
        });

        // if we went through all of that and didn't find a tag, we will
        // pretend there is no tag in view, and clear out all the LL pose
        // and targeting information
        if (closestTagInView == null) {
            posePublisher.accept(NO_TAG);
            txPublisher.accept(0.0);
            taPublisher.accept(0.0);
            tidPublisher.accept(0.0);
            tvPublisher.accept(0.0);
            return;
        }

        Pose2d tagPose = closestTagInView.pose.toPose2d();
        Translation2d tagTranslation = tagPose
                .relativeTo(odometryPose)
                .getTranslation();

        // if we do have a tag, we will fake the tag area using its distance
        // from the robot (+X) and the offset using it's left-right offset
        // from the robot (+/-Y)
        double tagArea = 1.0 / tagTranslation.getX();
        double tagOffset = tagTranslation.getY();

        // let's report the pose for the classic algorithm
        double [] classicPose = generateFakePoseInfo(
            odometryPose.getTranslation(),
            odometryPose.getRotation(),
            closestTagInView,
            Util.feetBetween(tagPose, odometryPose),
            tagArea);
        posePublisher.accept(classicPose);

        // we will report the mega tag algorithm with the same details,
        // but its error going in the opposite direction, to differentiate
        // it from the mega tag pose
        double [] megaTagPose = Arrays.copyOf(classicPose, classicPose.length);
        megaTagPose[0] -= 2.0 * poseError;
        megaTagPose[1] -= 2.0 * poseError;
        posePublisher.accept(megaTagPose);

        // and finally, we'll update the basic targeting info
        txPublisher.accept(tagOffset);
        taPublisher.accept(tagArea);
        tidPublisher.accept(closestTagInView.ID);
        tvPublisher.accept(1.0);
    }

    /*
     * This creates fake bot pose information for the robot, in the format that
     * the Limelight would generate it. This is what we use for pose estimation
     * in the Limelight.
     */
    private double [] generateFakePoseInfo(
                    Translation2d robotPosition, 
                    Rotation2d robotHeading,
                    AprilTag tag,
                    double distanceToTag,
                    double tagArea) {

        return new double [] {

            // first 3 = translation (X, Y, Z) in meters

            // the LL is calculating where it "thinks" the robot is by using 
            // vision recognition and AprilTag information. this is pretty good,
            // but always a little bit off; we'll simulate that by using a small
            // translation from the robot's current pose.( and we'll assume the
            // robot isn't going to leave the ground.)
            robotPosition.getX() + poseError,
            robotPosition.getY() + poseError,
            0.0,

            // next 3 = rotation in degrees (roll, pitch, yaw)
            // we don't tend to trust the LL rotation calculation, so here we'll
            // just leave it the same as reported by the robot
            0.0,
            0.0,
            robotHeading.getDegrees(),

            0.0, // total latency
            1.0, // tag count
            0.0, // tag span (we don't use this)
            distanceToTag, // average distance from camera
            tagArea, // average tag area
            tag.ID, // tag ID
            1.0, // horizontal tx to primary pixel
            1.0, // vertical tx to primary pixel
            tagArea, // tag t area
            distanceToTag, // distance to camera
            distanceToTag, // distance to robot
            0.3  // ambiguity

        };
    }
}
