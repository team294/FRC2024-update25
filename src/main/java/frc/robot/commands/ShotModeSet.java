// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.utilities.BCRRobotState;
import frc.robot.utilities.FileLog;
import frc.robot.utilities.BCRRobotState.ShotMode;

public class ShotModeSet extends InstantCommand {
  ShotMode shotMode;
  BCRRobotState robotState;
  FileLog log;

  /**
   * Record if shooter is in mode for a Far shot (lobbing note towards alliance partner)
   * @param shotMode 
   * @param robotState
   * @param log
   */
  public ShotModeSet(ShotMode shotMode, BCRRobotState robotState, FileLog log) {
    this.shotMode = shotMode;
    this.robotState = robotState;
    this.log = log;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    robotState.setShotMode(shotMode);
    log.writeLog(true, "ShotModeSet", "ShotMode", shotMode);
  }
}
