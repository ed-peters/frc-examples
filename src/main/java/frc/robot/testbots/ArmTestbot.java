package frc.robot.testbots;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.arm.ArmMotorSim;
import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.subsystems.arm.ArmSubsystem.ArmPreset;

public class ArmTestbot extends TimedRobot {

    ArmMotorSim sim;
    ArmSubsystem arm;
    CommandXboxController controller;
    double tuningSpeed;

    public ArmTestbot() {

        sim = new ArmMotorSim();
        arm = new ArmSubsystem(sim);
        controller = new CommandXboxController(0);

        arm.setDefaultCommand(arm.holdCommand());

        // a will run the tuning command with the specified velocity
        controller.a().whileTrue(arm.defer(() -> arm.constantVelocityCommand(tuningSpeed)));
        controller.b().whileTrue(arm.presetCommand(ArmPreset.LOW));
        controller.x().whileTrue(arm.presetCommand(ArmPreset.MIDDLE));
        controller.y().whileTrue(arm.presetCommand(ArmPreset.HIGH));

        SmartDashboard.putData("ArmSim", builder -> {
            builder.addDoubleProperty("Angle", sim::getAngle, sim::setAngle);
            builder.addDoubleProperty("Speed", () -> tuningSpeed, val -> tuningSpeed = val);
        });
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
    }
}
