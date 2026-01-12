package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import static frc.robot.util.Util.pref;

/**
 * Central class for all configuration properties. This includes both static
 * constants (stuff that won't change during the season), and "live" preferences
 * that we might be tweaking and tuning as we go.
 */
public interface Config {

//region Swerve ----------------------------------------------------------------

    /**
     * Hardware configuration for the swerve chassis, and general behavior
     * options for the swerve drive subsystem
     */
    interface SwerveSubsystem {

        /** Enable/disable cosine compensation */
        BooleanSupplier cosineCompensation = pref("Swerve/CosineCompensation?", false);

        /** Enable/disable using the fused pose estimate for driver assist */
        BooleanSupplier useFusedPose = pref("Swerve/UseFusedPose?", true);

        /** Enable/disable using the fused heading estimate of that pose */
        BooleanSupplier useFusedHeading = pref("Swerve/UseFusedHeading?", false);

        /** Top speeds in teleop (feet and degrees per second) */
        DoubleSupplier maxTeleopTranslate = pref("Swerve/TeleopMaxTranslate", 10.0);
        DoubleSupplier maxTeleopRotate = pref("Swerve/TeleopMaxRotate", 180.0);

        /** Top speeds for auto alignment commands */
        DoubleSupplier maxAlignTranslate = pref("Swerve/AlignMaxTranslate", 10.0);
        DoubleSupplier maxAlignRotate = pref("Swerve/AlignMaxRotate", 720.0);

        /**
         * Acceleration for auto alignment. It's easiest to think of these as
         * a time factor, so we just implement them that way here.
         */
        DoubleSupplier alignTimeToTopSpeed = pref("Swerve/AlignTimeToTopSpeed", 0.5);
        DoubleSupplier alignRotateAcceleration = () -> maxAlignRotate.getAsDouble() / alignTimeToTopSpeed.getAsDouble();
        DoubleSupplier alignTranslateAcceleration = () -> maxAlignTranslate.getAsDouble() / alignTimeToTopSpeed.getAsDouble();

        double WHEEL_DIAM_M = Units.inchesToMeters(3.0);
        double WHEEL_CIRC_M = WHEEL_DIAM_M * Math.PI;
        double WHEEL_BASE_M = Units.inchesToMeters(26.25);
        double TRACK_WIDTH_M = Units.inchesToMeters(26.5);
        SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
                new Translation2d(WHEEL_BASE_M / 2, TRACK_WIDTH_M / 2),
                new Translation2d(WHEEL_BASE_M / 2, -TRACK_WIDTH_M / 2),
                new Translation2d(-WHEEL_BASE_M / 2, TRACK_WIDTH_M / 2),
                new Translation2d(-WHEEL_BASE_M / 2, -TRACK_WIDTH_M / 2));
    }

    /**
     * Detailed configuration for swerve teleop options
     */
    interface SwerveTeleop {

        /** Toggle drive relative mode */
        BooleanSupplier driverRelative = pref("SwerveTeleop/DriverRelative?", true);

        /** Joystick input conditioning */
        DoubleSupplier deadband = pref("SwerveTeleop/Deadband", 0.1);
        DoubleSupplier exponent = pref("SwerveTeleop/Exponent", 2.0);

        /** Turbo/sniper multipliers */
        DoubleSupplier turboFactor = pref("SwerveTeleop/TurboFactor", 2.0);
        DoubleSupplier sniperFactor = pref("SwerveTeleop/SniperFactor", 0.5);
        BooleanSupplier applySniperToRotate = pref("SwerveTeleop/SniperOnRotate?", true);

        /**
         * Slew limiting (in "units per second) where units are in feet,
         * so a rate of 4.0 means you will hit 4 feet per second after 1
         * second)
         */
        BooleanSupplier applySlew = pref("SwerveTeleop/ApplySlew?", false);
        DoubleSupplier slewRate = pref("SwerveTeleop/SlewRate", 4.0);
    }

    /**
     * Configuration for the {@link frc.robot.commands.swerve.SwerveAutoPoseCommand}
     * and autonomous mode
     */
    interface SwerveAutoTuning {

        /** Feedback constants for autonomous mode */
        DoubleSupplier pathP = pref("SwerveAuto/Path/kP", 0.4);
        DoubleSupplier pathD = pref("SwerveAuto/Path/kD", 0.0);

        /** Feedback constants for angle correction during auto rotation */
        DoubleSupplier rotateP = pref("SwerveAuto/Rotate/kP", 0.4);
        DoubleSupplier rotateD = pref("SwerveAuto/Rotate/kD", 0.0);
        DoubleSupplier rotateTolerance = pref("SwerveAuto/Rotate/Tolerance", 1.0);

        /** Feedback constants for position correction during auto translation */
        DoubleSupplier translateP = pref("SwerveAuto/Translate/kP",2.0);
        DoubleSupplier translateD = pref("SwerveAuto/Translate/kD", 0.0);
        DoubleSupplier translateTolerance = pref("SwerveAuto/Translate/Tolerance", 2.0);
    }

//endregion

//region Vision ----------------------------------------------------------------

    interface LimelightSubsystem {

        /** Name of the Limelight camera in Network Tables */
        String limelightName = "limelight";

        /** Camera height (in inches) and angle (in degrees off horizontal) */
        double mountHeightAboveFloor = 6.0;
        double mountAngle = 15.0;

        /** How far (in feet) away from a tag for auto alignment */
        DoubleSupplier feetInFrontOfTag = pref("Limelight/FeetInFrontOfTag", 5.0);

    }

    /**
     *  Configuration for the {@link frc.robot.commands.vision.LimelightTranslateCommand}
     */
    interface LimelightTranslateTuning {

        /**
         * Feedback configuration for forward/back translation; desired area
         * will determine how far back from the target you wind up being when
         * you are done
         */
        DoubleSupplier areaP = pref("LimelightTranslate/AreaKP", 2.0);
        DoubleSupplier areaD = pref("LimelightTranslate/AreaKD", 0.1);
        DoubleSupplier areaTolerance = pref("LimelightTranslate/AreaTolerance", 0.2);
        DoubleSupplier desiredArea = pref("LimelightTranslate/DesiredArea", 1.8);

        /**
         * Feedback configuration for left/right translation; desired offset
         * will determine how far left/right of the target you wind up being
         * when you are done
         */
        DoubleSupplier offsetP = pref("LimelightTranslate/OffsetKP", 2.0);
        DoubleSupplier offsetD = pref("LimelightTranslate/OffsetKD", 0.1);
        DoubleSupplier offsetTolerance = pref("LimelightTranslate/OffsetTolerance", 3.0);
        DoubleSupplier desiredOffset = pref("LimelightTranslate/DesiredOffset", 0.0);
    }

    /**
     *  Configuration for the {@link frc.robot.commands.vision.LimelightRotateCommand}
     */
    interface LimelightRotateTuning {

        /** Feedback configuration for rotation */
        DoubleSupplier rotateP = pref("LimelightRotate/RotateKP", 0.1);
        DoubleSupplier rotateD = pref("LimelightRotate/RotateKD", 0.01);
        DoubleSupplier rotateTolerance = pref("LimelightRotate/RotateTolerance", 2.0);

        /** Feedback configuration for forward/back translation */
        DoubleSupplier distanceP = pref("LimelightRotate/DistanceKP", 1.0);
        DoubleSupplier distanceD = pref("LimelightRotate/DistanceKD", 0.1);
        DoubleSupplier distanceTolerance = pref("LimelightRotate/DistanceTolerance", 0.5);
    }

//endregion

//region Intake ----------------------------------------------------------------

    interface Intake {

        /** Physical properties of the mechanism */
        double gearRatio = 1.0 / 3.0;
        double wheelDiameter = 4.0 / 12.0;
        double wheelCircumference = wheelDiameter * Math.PI;

        /** Feedforward/feedback constants for speed */
        DoubleSupplier p = pref("IntakeSubsystem/kP", 0.0);
        DoubleSupplier v = pref("IntakeSubsystem/kV", 0.0);
        DoubleSupplier tolerance = pref("IntakeSubsystem/Tolerance", 0.0);

        /** Preset speeds in feet per seconds */
        DoubleSupplier gobbleSpeed = pref("IntakePresets/Gobble", 10.0);
        DoubleSupplier indexSpeed = pref("IntakePresets/Index", 20.0);
        DoubleSupplier ejectSpeed = pref("IntakePresets/Eject", 3.0);

    }

//endregion

//region Arm -------------------------------------------------------------------

    interface Arm {

        /** Physical properties of the mechanism */
        double gearRatio = 1.0 / (75.0 / (24.0 / 64.0));
        double degreesPerMotorRev = 360.0 * gearRatio;

        /** Constraints */
        DoubleSupplier minAngle = pref("ArmSubsystem/MinAngle", -20.0);
        DoubleSupplier maxAngle = pref("ArmSubsystem/MaxAngle", 80.0);
        DoubleSupplier maxVelocity = pref("ArmSubsystem/MaxVelocity", 90.0);

        /**
         * Acceleration. It's easiest to think of these as
         * a time factor, so we just implement them that way here.
         */
        DoubleSupplier timeToTopSpeed = pref("ArmSubsystem/TimeToTopSpeed", 0.5);
        DoubleSupplier maxAcceleration = () -> maxVelocity.getAsDouble() / timeToTopSpeed.getAsDouble();

        /** Feedforward/feedback constants for speed */
        DoubleSupplier p = pref("ArmSubsystem/kP", 0.0);
        DoubleSupplier d = pref("ArmSubsystem/kD", 0.0);
        DoubleSupplier g = pref("ArmSubsystem/kG", 0.0);
        DoubleSupplier v = pref("ArmSubsystem/kV", 0.0);
        DoubleSupplier maxFeedback = pref("ArmSubsystem/MaxFeedback", 1.0);
        DoubleSupplier tolerance = pref("ArmSubsystem/Tolerance", 0.0);

        /** Preset speeds in feet per seconds */
        DoubleSupplier angleHigh = pref("ArmPresets/High", 10.0);
        DoubleSupplier angleMiddle = pref("ArmPresets/Middle", 20.0);
        DoubleSupplier angleLow = pref("ArmPresets/Low", 3.0);

    }

//endregion

}
