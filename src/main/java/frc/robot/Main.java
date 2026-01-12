// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.testbots.ArmTestbot;
import frc.robot.testbots.IntakeTestbot;
import frc.robot.testbots.LimelightTestbot;
import frc.robot.testbots.PoseMathTestbot;
import frc.robot.testbots.SwerveTestbot;

public final class Main {
  private Main() {}

  public static void main(String... args) {
    RobotBase.startRobot(SwerveTestbot::new);
//    RobotBase.startRobot(ArmTestbot::new);
//    RobotBase.startRobot(IntakeTestbot::new);
//    RobotBase.startRobot(PoseMathTestbot::new);
//    RobotBase.startRobot(LimelightTestbot::new);
  }
}
