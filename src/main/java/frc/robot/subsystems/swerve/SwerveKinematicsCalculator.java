package frc.robot.subsystems.swerve;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

import java.util.Objects;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

/**
 * Converts {@link ChassisSpeeds} into {@link SwerveModuleState}s that will
 * move the robot in the desired manner. Implements the following features:
 * <ul>
 *
 *     <li>Optimization minimizes the amount of turning each wheel has to do
 *     by reversing the wheel direction if it means less turning</li>
 *
 *     <li>Cosine compensation reduces the speed of the wheel if it's not yet
 *     pointing in the desired direction, to reduce the amount of "skew"
 *     when changing directions</li>
 *
 *     <li>Movement around a custom center of rotation (this allows your
 *     robot to "orbit" a fixed point such as a target on the field)</li>
 *
 * </ul>
 *
 * <p>See the <a href="https://docs.wpilib.org/en/stable/docs/software/kinematics-and-odometry/swerve-drive-kinematics.html">WPILib
 * docs</a> for more background on kinematics.</p>
 *
 * <p>This class is implemented so it doesn't depend on a specific swerve
 * drive implementation or configuration; if it proves useful we might
 * later move it into a library.</p>
 */
public class SwerveKinematicsCalculator {

    final SwerveDriveKinematics kinematics;
    final Supplier<SwerveModulePosition[]> modulePositionGetter;
    final BooleanSupplier useCosineCompensation;

    /**
     * Creates a {@link SwerveKinematicsCalculator}
     * @param kinematics the chassis kinematics (required)
     * @param modulePositionGetter a getter for module positions (required)
     * @param useCosineCompensation a getter for whether we should use cosine compensation
     * @throws IllegalArgumentException if required parameters are null
     */
    public SwerveKinematicsCalculator(SwerveDriveKinematics kinematics,
                                      Supplier<SwerveModulePosition[]> modulePositionGetter,
                                      BooleanSupplier useCosineCompensation) {
        this.kinematics = Objects.requireNonNull(kinematics);
        this.modulePositionGetter = Objects.requireNonNull(modulePositionGetter);
        this.useCosineCompensation = useCosineCompensation == null
                ? () -> false
                : useCosineCompensation;
    }

    /**
     * @param speeds the desired chassis speeds
     * @return the required swerve module states, calculated with the center
     * of the robot as the center of rotation
     */
    public SwerveModuleState [] calculateStates(ChassisSpeeds speeds) {
        return calculateStates(speeds, Translation2d.kZero);
    }

    /**
     * @param speeds the desired chassis speeds
     * @param centerOfRotation the center of rotation (null means center of robot)
     * @return the required swerve module states, calculated with the supplied
     * center of rotation
     */
    public SwerveModuleState [] calculateStates(ChassisSpeeds speeds,
                                                Translation2d centerOfRotation) {

        // no center of rotation? assume it's the center of the robot
        if (centerOfRotation == null) {
            centerOfRotation = Translation2d.kZero;
        }

        // normal kinematic calculations
        SwerveModuleState [] states = kinematics.toSwerveModuleStates(
                speeds,
                centerOfRotation);

        SwerveModulePosition [] positions = modulePositionGetter.get();

        // optimize
        for (int i=0; i<states.length; i++) {
            states[i].optimize(positions[i].angle);
        }

        // cosine compensation
        if (useCosineCompensation.getAsBoolean()) {
            for (int i=0; i<states.length; i++) {
                states[i].speedMetersPerSecond *= states[i].angle
                        .minus(positions[i].angle)
                        .getCos();
            }
        }

        return states;
    }
}
