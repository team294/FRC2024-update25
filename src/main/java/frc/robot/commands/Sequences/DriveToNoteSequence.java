// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Sequences;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.DriveToNote;
import frc.robot.commands.ShooterSetPercent;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Wrist;
import frc.robot.utilities.BCRRobotState;
import frc.robot.utilities.FileLog;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class DriveToNoteSequence extends SequentialCommandGroup {
  /** Creates a new DriveToNoteSequence. */
  public DriveToNoteSequence(Intake intake, Shooter shooter, Feeder feeder, Wrist wrist, DriveTrain drivetrain, BCRRobotState robotState, FileLog log) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new ShooterSetPercent(-0.08, shooter, log),
      new ParallelCommandGroup(
        new IntakePiece(intake, feeder, wrist, shooter, robotState, log),
        new DriveToNote(feeder, drivetrain, log)
      )
    );
  }
}
