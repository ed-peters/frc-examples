package frc.robot.subsystems.swerve;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.util.Util;

import java.util.Objects;

/**
 * Implements a fake swerve drive. This lets you run swerve-related code in
 * simulation.
 */
public class SimSwerveChassis implements SwerveHardware {

    final SwerveDriveKinematics kinematics;
    double currentHeading;
    double [] velocity;
    double [] angle;
    double [] distance;

    /**
     * Creates a {@link SimSwerveChassis}
     * @param kinematics kinematics for the drive (required)
     * @throws IllegalArgumentException if required parameters are null
     */
    public SimSwerveChassis(SwerveDriveKinematics kinematics) {
        this.kinematics = Objects.requireNonNull(kinematics);
        this.currentHeading = 0.0;
        this.velocity = new double[]{ 0.0, 0.0, 0.0, 0.0 };
        this.angle = new double[]{ 0.0, 0.0, 0.0, 0.0 };
        this.distance = new double[]{ 0.0, 0.0, 0.0, 0.0 };
    }

    @Override
    public Rotation2d getHeading() {
        return Rotation2d.fromRadians(currentHeading);
    }

    @Override
    public ChassisSpeeds getCurrentSpeed() {

        // we use kinematics to turn the current state of each module into
        // a chassis speed on the field
        SwerveModuleState[] states = new SwerveModuleState[4];
        for (int i=0; i<4; i++) {
            states[i] = new SwerveModuleState(
                    velocity[i],
                    Rotation2d.fromRadians(angle[i]));
        }
        return kinematics.toChassisSpeeds(states);
    }

    @Override
    public SwerveModulePosition [] getModulePositions() {

        // module positions are just the current internal state
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for (int i=0; i<4; i++) {
            positions[i] = new SwerveModulePosition(distance[i], Rotation2d.fromRadians(angle[i]));
        }
        return positions;
    }

    @Override
    public void setModuleStates(SwerveModuleState [] states) {

        Objects.requireNonNull(states);

        for (int i=0; i<4; i++) {

            // record current velocity and angle
            velocity[i] = states[i].speedMetersPerSecond;
            angle[i] = states[i].angle.getRadians();

            // calculate how far we've rolled in this unit time
            distance[i] += states[i].speedMetersPerSecond * Util.DT;
        }

        // calculate how much the robot's heading has changed as a result of
        // this motion, and use the difference between the last value and
        // this new value to calculate the yaw rate
        ChassisSpeeds speeds = kinematics.toChassisSpeeds(states);
        currentHeading = MathUtil.angleModulus(currentHeading + speeds.omegaRadiansPerSecond * Util.DT);
    }
}
