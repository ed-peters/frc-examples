package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.util.RateCalculator;
import frc.robot.subsystems.vision.LimelightHelpers.PoseEstimate;

import java.util.Objects;
import java.util.function.Supplier;

/**
 * Represents a Limelight camera and implements utility methods and some logic
 * from the <a href="https://docs.limelightvision.io/docs/docs-limelight/tutorials/tutorial-swerve-pose-estimation">turotial</a>.
 */
@SuppressWarnings("all")
public class Limelight {

    final String limelightName;
    final RateCalculator yawCalculator;

    /**
     * Creates a {@link Limelight}
     * @param limelightName the name of the camera in Network Tables
     * @param headingSupplier supplies the robot's current heading for pose estimation
     */
    public Limelight(String limelightName,
                     Supplier<Rotation2d> headingSupplier) {
        Objects.requireNonNull(headingSupplier);
        this.limelightName = limelightName == null ? "limelight": limelightName;
        this.yawCalculator = new RateCalculator(() -> headingSupplier.get().getRadians());
    }

    //region Targeting -------------------------------------------------------------

    /**
     * Represents target information from the current pipeline. Note that
     * a target could be an AprilTag, but doesn't have to be (depending on
     * the current pipeline).
     *
     * @param area   target area as a percentage of the image (0 to 100)
     * @param offset horizontal offset of the target in the image (-29.8 to 29.8),
     *               with negative values meaning the target is to the left of
     *               center
     * @param angle  vertical offset of the target from the crosshair in degrees
     * @param tagId  ID of the primary in-view AprilTag (-1 if the target isn't
     *               a tag)
     */
    public static record LimelightTarget(double area, double offset,
                                         double angle, int tagId) {

        /** @return is this a tag? */
        public boolean isAprilTag() {
            return tagId > 0;
        }
    }

    /**
     * @return a {@link LimelightTarget} representing the current
     * in-view target for that camera (null if there isn't one)
     */
    public LimelightTarget getTarget() {

        // the camera will tell us if there's a target in view or not
        if (!LimelightHelpers.getTV(limelightName)) {
            return null;
        }

        // these are the relevant keys for the targeting data we want;
        // if the camera claims to have a target and one of these isn't
        // set, it's an error
        double ta = LimelightHelpers.getTA(limelightName);
        double tx = LimelightHelpers.getTX(limelightName);
        double ty = LimelightHelpers.getTY(limelightName);
        if (Double.isNaN(ta) || Double.isNaN(tx) || Double.isNaN(ty)) {
            return null;
        }

        // we want the offset to be negative when the tag is to the left (so
        // positive motion will decrease it). this is the opposite of how the
        // camera reports it, so we will negate it before returning
        return new LimelightTarget(
                ta,
                -tx,
                ty,
                (int) LimelightHelpers.getFiducialID(limelightName));
    }

//endregion

//region LED mode --------------------------------------------------------------

    /**
     * Represents the different LED modes for the camera (note that the order
     * of this is important - it corresponds to the values of the "ledMode"
     * in the <a href="https://docs.limelightvision.io/docs/docs-limelight/apis/complete-networktables-api#:~:text=ledMode">vendor
     * documentation</a>.
     */
    public static enum LedMode {
        PIPELINE, ON, OFF, BLINK
    }

    /**
     * Set the LED mode for a Limelight camera
     * @param mode the desired light mode (null means {@link LedMode#PIPELINE})
     */
    public void setLedMode(LedMode mode) {
        if (mode == null) {
            mode = LedMode.PIPELINE;
        }
        switch (mode) {
            case PIPELINE:
                LimelightHelpers.setLEDMode_PipelineControl(limelightName);
                break;
            case ON:
                LimelightHelpers.setLEDMode_ForceOn(limelightName);
                break;
            case OFF:
                LimelightHelpers.setLEDMode_ForceOff(limelightName);
                break;
            case BLINK:
                LimelightHelpers.setLEDMode_ForceBlink(limelightName);
                break;
        }
    }

//endregion

//region Pipeline selection ----------------------------------------------------

    /**
     * Represents information about a pipeline
     * @param index the index of the pipeline in Limelight configuration
     * @param type the type of pipeline
     */
    public static record Pipeline(int index, String type) {

    }

    /**
     * @return the current pipeline index
     */
    public Pipeline getPipeline() {
        int idx = (int) LimelightHelpers.getCurrentPipelineIndex(limelightName);
        String type = LimelightHelpers.getCurrentPipelineType(limelightName);
        return new Pipeline(idx, type);
    }

    /**
     * Sets the targeting pipeline to use
     * @param pipelineIndex the desired pipeline index
     */
    public void setPipelineIndex(int pipelineIndex) {
        LimelightHelpers.setPipelineIndex(limelightName, pipelineIndex);
    }

//endregion

//region Pose estimates --------------------------------------------------------

    /**
     * Maximum allowable yaw rate for the robot - if we're spinning any
     * faster than this, we won't trust pose estimates. This is the value
     * recommended in the
     * <a href="https://docs.limelightvision.io/docs/docs-limelight/tutorials/tutorial-swerve-pose-estimation">tutorial</a>.
     */
    public static final double MAX_YAW_RATE = 720.0;

    /**
     * Gets the current pose estimate using the MegaTag2 algorithm. This
     * implements the logic in the
     * <a href="https://docs.limelightvision.io/docs/docs-limelight/tutorials/tutorial-swerve-pose-estimation">tutorial</a>
     * for determining whether to trust the estimate.
     *
     * @param currentHeading the robot's current heading
     * @return the current pose estimate; null if there isn't one trustworthy
     * enough to use
     */
    public PoseEstimate getPoseEstimate() {

        // yaw rate calculations are vital - the first time through, we're just
        // going to remember the current yaw and bail out. subsequent calls will
        // let us calculate the rate.
        double currentYawRate = yawCalculator.calculate();
        if (Double.isNaN(currentYawRate)) {
            return null;
        }

        // the LL always wants to know where the robot is facing (this helps
        // make MegaTag2 more accurate)
        LimelightHelpers.SetRobotOrientation(limelightName, 0.0,
                currentYawRate, 0.0, 0.0, 0.0, 0.0);

        // if we're spinning too quickly, we ignore the estimate
        if (Math.abs(currentYawRate) > MAX_YAW_RATE) {
            return null;
        }

        PoseEstimate estimate = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limelightName);
        if (estimate == null || estimate.tagCount == 0) {
            return null;
        }

        return estimate;
    }

//endregion

}
