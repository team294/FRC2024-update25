// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Sequences;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.FeederSetPercent;
import frc.robot.commands.FileLogWrite;
import frc.robot.commands.IntakeSetPercent;
import frc.robot.commands.RobotStateSetIdle;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Intake;
import frc.robot.utilities.BCRRobotState;
import frc.robot.utilities.BCRRobotState.State;
import frc.robot.utilities.FileLog;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class IntakeStopState extends SequentialCommandGroup {
  /** Creates a new IntakeStopState. */
  public IntakeStopState(Feeder feeder, Intake intake, BCRRobotState robotState, FileLog log) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new ConditionalCommand(
        new SequentialCommandGroup(
          new IntakeSetPercent(0, 0, intake, log),
          new FeederSetPercent(0, feeder, log),
          new RobotStateSetIdle(robotState, feeder, log)
        ),
        new FileLogWrite(false, false, "Intake Stop State", "Not in intake to feeder state", log), 
        () -> robotState.getState() == State.INTAKING)
      
    );
  }
}
