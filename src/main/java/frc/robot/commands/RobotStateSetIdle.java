// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Feeder;
import frc.robot.utilities.FileLog;
import frc.robot.utilities.BCRRobotState;

public class RobotStateSetIdle extends SequentialCommandGroup {

  /**
   * Sets the robot state to IDLE
   * <p> Note that this command does <b>not</b> require the feeder subsystem.
   * @param robotState
   * @param feeder
   * @param log
   */
  public RobotStateSetIdle(BCRRobotState robotState, Feeder feeder, FileLog log) {
    addCommands(
      new RobotStateSet(BCRRobotState.State.IDLE, robotState, log)
    );
  }
}
