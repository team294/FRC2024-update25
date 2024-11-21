// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Sequences;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.FeederConstants;
import frc.robot.commands.*;
import frc.robot.subsystems.Intake;
import frc.robot.utilities.BCRRobotState;
import frc.robot.subsystems.Feeder;
import frc.robot.utilities.FileLog;

public class IntakePieceAuto extends SequentialCommandGroup {

  /**
   * Turns on the intake and feeder motors, and sets the robot state
   * to INTAKE_TO_FEEDER.
   * <p><b>Note:</b> This sequence does not stop intaking!  This requires a 
   * trigger to turn off intaking when a piece gets to the feeder, or the
   * driver can manually turn off intaking.
   * @param intake
   * @param feeder
   * @param robotState
   * @param log
   */
  public IntakePieceAuto(Intake intake, Feeder feeder, BCRRobotState robotState, FileLog log) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());

    addCommands(
      new RobotStateSet(BCRRobotState.State.INTAKING, robotState, log),
      new IntakeSetPercent(IntakeConstants.intakePercent,IntakeConstants.centeringPercent, intake, log),
      new FeederSetPercent(FeederConstants.feederPercent, feeder, log),
      new WaitCommand(10).until(() -> feeder.isPiecePresent()),
      new IntakeSetPercent(0, 0, intake, log),
      new FeederSetPercent(-0.05, feeder, log),
      new WaitCommand(0.1),
      new ParallelCommandGroup(
        new FeederSetPercent(0.0, feeder, log),      
        new RobotStateSetIdle(robotState, feeder, log)
      )
    );
  }
}
