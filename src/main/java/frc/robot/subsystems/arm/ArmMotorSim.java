package frc.robot.subsystems.arm;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.util.Util;

import java.util.function.DoubleConsumer;

import static frc.robot.Config.Arm.degreesPerMotorRev;
import static frc.robot.Config.Arm.gearRatio;
import static frc.robot.Config.Arm.maxAngle;
import static frc.robot.Config.Arm.minAngle;

/**
 * Implements the {@link ArmHardware} interface on top of a
 * {@link SingleJointedArmSim} for testing in simulation.
 */
public class ArmMotorSim implements ArmHardware {

    final SingleJointedArmSim sim;
    final DoubleConsumer widget;

    // these configuration values are based on our 2024 Crescendo robot; they
    // represent a heavy weight on the end of the meter-long arm
    public ArmMotorSim() {
        this.sim = new SingleJointedArmSim(
                DCMotor.getNEO(2),
                1.0 / gearRatio,
                SingleJointedArmSim.estimateMOI(1.0, 10.0),
                1.0,
                0.0,
                Math.toRadians(maxAngle.getAsDouble()),
                true,
                0.0);
        this.widget = createWidget();
    }

    /**
     * Creates the dashboard widget, and returns a {@link DoubleConsumer} that
     * can be used to set its height. The consumer accepts a height in inches,
     * scales that to the height of the canvas, and adjusts the widget
     * accordingly.
     */
    private DoubleConsumer createWidget() {

        // creates a horizontal line widget in the dashboard
        Mechanism2d mech = new Mechanism2d(4, 4);
        MechanismRoot2d root = mech.getRoot("ArmSim", 0.5, 0.5);
        MechanismLigament2d handle = root.append(new MechanismLigament2d("riser", 0.5, 90.0, 1, new Color8Bit(Color.kBlack)));
        handle.append(new MechanismLigament2d("platform", 3.0, -90, 1, new Color8Bit(Color.kWhite)));

        // add it to the dashboard
        SmartDashboard.putData("ArmSim", mech);

        // returns a function that will set the line's height
        return value -> {
            double num = (value - minAngle.getAsDouble());
            double den = (maxAngle.getAsDouble() - minAngle.getAsDouble());
            handle.setLength(MathUtil.clamp(num / den, 0.0, 1.0) * 3.0);
        };
    }

    @Override
    public boolean isBrakeEnabled() {
        return false;
    }

    @Override
    public void setBrake(boolean brake) {
        Util.log("[arm-sim] we don't support toggling the brake!");
    }

    @Override
    public double getAngle() {
        return Units.radiansToDegrees(sim.getAngleRads());
    }

    /**
     * This allows setting the height of the simulated arm directly from the
     * dashboard, which is useful in testing
     * @param angle target angle in degrees
     */
    public void setAngle(double angle) {
        sim.setState(Math.toRadians(angle), 0.0);
    }

    @Override
    public double getVelocity() {
        return Units.radiansToDegrees(sim.getVelocityRadPerSec()) / degreesPerMotorRev;
    }

    @Override
    public double getMotorAmps() {
        return sim.getCurrentDrawAmps();
    }

    @Override
    public void applyVolts(double volts) {
        sim.setInputVoltage(volts);
        sim.update(Util.DT);
        widget.accept(Units.radiansToDegrees(sim.getAngleRads()));
    }
}
