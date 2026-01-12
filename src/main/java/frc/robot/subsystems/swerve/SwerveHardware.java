package frc.robot.subsystems.swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

import java.util.Objects;
import java.util.function.Consumer;
import java.util.function.Supplier;

/**
 * Interface for the hardware of a swerve drive and gyro
 */
public interface SwerveHardware {

    /** @return the current heading of the robot (usually comes from gyro) */
    Rotation2d getHeading();

    /** @return the current speed of the robot */
    ChassisSpeeds getCurrentSpeed();

    /** @return the positions of each of the swerve modules */
    SwerveModulePosition[] getModulePositions();

    /**
     * Updates the states of the modules
     * @param states new module states (required)
     * @throws IllegalArgumentException if required parameters are null
     */
    void setModuleStates(SwerveModuleState [] states);

    /**
     * Creates a new hardware implementation using the supplied functions
     * @param gyroHeadingSupplier supplier for heading (required)
     * @param currentSpeedSupplier supplier for speeds (required)
     * @param modulePositionSupplier supplier for module positions (required)
     * @param moduleStateConsumer consumer of module states (required)
     * @return the new implementation
     * @throws IllegalArgumentException if required parameters are null
     */
    static SwerveHardware create(Supplier<Rotation2d> gyroHeadingSupplier,
                                 Supplier<ChassisSpeeds> currentSpeedSupplier,
                                 Supplier<SwerveModulePosition[]> modulePositionSupplier,
                                 Consumer<SwerveModuleState[]> moduleStateConsumer) {

        Objects.requireNonNull(gyroHeadingSupplier);
        Objects.requireNonNull(currentSpeedSupplier);
        Objects.requireNonNull(modulePositionSupplier);
        Objects.requireNonNull(moduleStateConsumer);

        return new SwerveHardware() {

            @Override
            public Rotation2d getHeading() {
                return gyroHeadingSupplier.get();
            }

            @Override
            public ChassisSpeeds getCurrentSpeed() {
                return currentSpeedSupplier.get();
            }

            @Override
            public SwerveModulePosition[] getModulePositions() {
                return modulePositionSupplier.get();
            }

            @Override
            public void setModuleStates(SwerveModuleState[] states) {
                moduleStateConsumer.accept(states);
            }
        };
    }
}
