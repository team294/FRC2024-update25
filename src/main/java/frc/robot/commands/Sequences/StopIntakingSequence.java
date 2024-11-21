// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Sequences;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.FeederConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.commands.*;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Intake;
import frc.robot.utilities.BCRRobotState;
import frc.robot.utilities.FileLog;

public class StopIntakingSequence extends ParallelCommandGroup {
  
  /**
   * Stops the intaking sequence and sets the robot state to IDLE
   * @param feeder Feeder subsystem
   * @param intake Intake subsystem
   * @param robotState Object with current robot state
   * @param log
   */
  public StopIntakingSequence(Feeder feeder, Intake intake, BCRRobotState robotState, FileLog log) {
    addCommands(
      new SequentialCommandGroup(
        new IntakeSetPercent(-IntakeConstants.intakePercent, IntakeConstants.centeringPercent, intake, log),
        new WaitCommand(0.250),
        new IntakeSetPercent(0.0, 0.0, intake, log)
      ),
      new SequentialCommandGroup(
        new FeederSetPercent(FeederConstants.feederBackPiecePercent, feeder, log),
        new WaitCommand(FeederConstants.feederBackPieceTime),
        new FeederSetPercent(0.0, feeder, log)
      ),
      new RobotStateSetIdle(robotState, feeder, log)
    );
  }
}
