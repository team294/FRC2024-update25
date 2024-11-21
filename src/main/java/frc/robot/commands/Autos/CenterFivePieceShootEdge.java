// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autos;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.WristConstants.WristAngle;
import frc.robot.commands.WristSetAngle;
import frc.robot.commands.Sequences.DriveBackAndSetWristAuto;
import frc.robot.commands.Sequences.DriveToAndIntakeNoteAuto;
import frc.robot.commands.Sequences.ScoreNoteAuto;
import frc.robot.commands.Sequences.ShootPiece;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Wrist;
import frc.robot.utilities.AllianceSelection;
import frc.robot.utilities.BCRRobotState;
import frc.robot.utilities.FileLog;
import frc.robot.utilities.TrajectoryCache;
import frc.robot.utilities.TrajectoryCache.TrajectoryType;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class CenterFivePieceShootEdge extends SequentialCommandGroup {
  /** Creates a new CenterFivePieceShoot. */
  public CenterFivePieceShootEdge(Intake intake, Wrist wrist, Shooter shooter, DriveTrain driveTrain, Feeder feeder, BCRRobotState robotState, TrajectoryCache cache, AllianceSelection alliance, FileLog log) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      //Shoots note
      new ScoreNoteAuto(WristAngle.speakerShotFromSpeaker, feeder, shooter, wrist, intake, robotState, log),

      //Drive to and intake near source-side note
      new DriveToAndIntakeNoteAuto(new Pose2d(0.4, 2.65, Rotation2d.fromDegrees(0)), new Pose2d(0.4, 5.55, Rotation2d.fromDegrees(0)), TrajectoryType.driveCenterStartToSourceNear, driveTrain, feeder, shooter, wrist, intake, robotState, cache, alliance, log),
      
      //Shoots note
      new ScoreNoteAuto(WristAngle.sourceCloseNoteShot, feeder, shooter, wrist, intake, robotState, log),

      //Drives to and intakes center note
      new DriveToAndIntakeNoteAuto(TrajectoryType.driveFromSourceNoteToCenterNote, driveTrain, feeder, shooter, wrist, intake, robotState, cache, alliance, log),

      //Scores note
      new ScoreNoteAuto(WristAngle.centerCloseNoteShot, feeder, shooter, wrist, intake, robotState, log),

      //Drives to and intakes amp-side note
      new DriveToAndIntakeNoteAuto(TrajectoryType.driveFromCenterNoteToAmpNote, driveTrain, feeder, shooter, wrist, intake, robotState, cache, alliance, log),

      //Scores note
      new ScoreNoteAuto(WristAngle.ampCloseNoteShot, feeder, shooter, wrist, intake, robotState, log),

      //Drives to and intakes far note
      new DriveToAndIntakeNoteAuto(TrajectoryType.driveAmpNoteToFarNote, driveTrain, feeder, shooter, wrist, intake, robotState, cache, alliance, log),

      //Drives back to score
      new DriveBackAndSetWristAuto(TrajectoryType.driveFromAmpFarToShootingPos, WristAngle.endFiveNoteShot, driveTrain, feeder, shooter, wrist, intake, robotState, cache, alliance, log),

      //Scores note
      new ShootPiece(true, shooter, feeder, wrist, robotState, log),

      //Prepares wrist to intake in teleop
      new WristSetAngle(WristAngle.lowerLimit, wrist, log)
    );
  }
}
