package frc.robot.testbots;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.intake.IntakeSim;
import frc.robot.subsystems.intake.IntakeSubsystem;

public class IntakeTestbot extends TimedRobot {

    IntakeSubsystem intake;
    CommandXboxController controller;

    public IntakeTestbot() {

        intake = new IntakeSubsystem(new IntakeSim());
        controller = new CommandXboxController(0);

        intake.setDefaultCommand(intake.idleCommand());

        // a will rotate a little bit left (spam it to turn around fully)
        controller.a().whileTrue(intake.wheelSpeedCommand("l1", 10.0));
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
    }
}
