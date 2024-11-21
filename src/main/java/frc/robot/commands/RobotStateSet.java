// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.utilities.BCRRobotState;
import frc.robot.utilities.FileLog;

public class RobotStateSet extends InstantCommand {
  BCRRobotState robotState;
  BCRRobotState.State newState;
  FileLog log;

  /**
   * Immediately sets the Robot State object to the given state.
   * This will update wherever the object is used.
   * @param newState State to set (ex: IDLE)
   * @param robotState robotState object
   * @param log log
   */
  public RobotStateSet(BCRRobotState.State newState, BCRRobotState robotState, FileLog log) {
    this.newState = newState;
    this.robotState = robotState;
    this.log = log;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    robotState.setState(newState);
    log.writeLog(true, "RobotStateSet", newState.toString());
  }
}
  