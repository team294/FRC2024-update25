// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autos;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.WristConstants.WristAngle;
import frc.robot.commands.Sequences.*;
import frc.robot.subsystems.*;
import frc.robot.utilities.AllianceSelection;
import frc.robot.utilities.BCRRobotState;
import frc.robot.utilities.FileLog;
import frc.robot.utilities.TrajectoryCache;
import frc.robot.utilities.TrajectoryCache.TrajectoryType;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SourceThreePieceShoot extends SequentialCommandGroup {
  /** Creates a new CenterSourceThreePieceShoot. 
   *  Note: Wrist angles for scoring have not been well 
   *  tested, will likely need more calibration (Outdated)
  */
  public SourceThreePieceShoot(Intake intake, Shooter shooter, DriveTrain driveTrain, Feeder feeder, Wrist wrist, BCRRobotState robotState, TrajectoryCache cache, AllianceSelection alliance, FileLog log) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      //Scores note
      new ScoreNoteAuto(WristAngle.speakerShotFromSpeaker, feeder, shooter, wrist, intake, robotState, log),
      new ShootPiece(ShooterConstants.shooterVelocityTop, ShooterConstants.shooterVelocityBottom, false, shooter, feeder, wrist, robotState, log),

      //Drives to and intakes the close source-side note
      new DriveToAndIntakeNoteAuto(TrajectoryType.driveToSourceCloseNote, driveTrain, feeder, shooter, wrist, intake, robotState, cache, alliance, log),

      //Scores note
      new ScoreNoteAuto(WristAngle.sourceCloseNoteShot, feeder, shooter, wrist, intake, robotState, log),
      new ShootPiece(ShooterConstants.shooterVelocityTop, ShooterConstants.shooterVelocityBottom, false, shooter, feeder, wrist, robotState, log),

      //Drives to and intakes the far note
      new DriveToAndIntakeNoteAuto(TrajectoryType.driveSourceNoteToFarNote, driveTrain, feeder, shooter, wrist, intake, robotState, cache, alliance, log),

      //Auto needs more testing / new trajectory to drive back past this point

      //Scores note
      new ScoreNoteAuto(WristAngle.sourceCloseNoteShot, feeder, shooter, wrist, intake, robotState, log),
      new ShootPiece(ShooterConstants.shooterVelocityTop, ShooterConstants.shooterVelocityBottom, true, shooter, feeder, wrist, robotState, log)
    );
  }
}
