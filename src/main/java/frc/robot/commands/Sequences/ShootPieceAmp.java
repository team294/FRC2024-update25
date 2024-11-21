// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Sequences;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.FeederConstants;
import frc.robot.commands.FeederSetPercent;
import frc.robot.commands.RobotStateSet;
import frc.robot.commands.RobotStateSetIdle;
import frc.robot.subsystems.Feeder;
import frc.robot.utilities.BCRRobotState;
import frc.robot.utilities.FileLog;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ShootPieceAmp extends SequentialCommandGroup {
  /** Creates a new ShootPieceAmp. */
  public ShootPieceAmp(Feeder feeder, BCRRobotState robotState, FileLog log) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new RobotStateSet(BCRRobotState.State.SHOOTING, robotState, log),
      new FeederSetPercent(FeederConstants.feederAmpShot, feeder, log),
      new WaitCommand(3).until(() -> !feeder.isPiecePresent()),
      new WaitCommand(1),
      new FeederSetPercent(0.0, feeder, log),
      new RobotStateSetIdle(robotState, feeder, log)
    );
  }
}
