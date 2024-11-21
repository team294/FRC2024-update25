// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autos;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.WristConstants.WristAngle;
import frc.robot.commands.*;
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
public class SourceTwoPieceShoot extends SequentialCommandGroup {
  /** Creates a new SourceTwoPieceShoot. */
  public SourceTwoPieceShoot(Intake intake, Wrist wrist, Shooter shooter, DriveTrain driveTrain, Feeder feeder, BCRRobotState robotState, TrajectoryCache cache, AllianceSelection alliance, FileLog log) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      //Sets wrist and scores note
      new ScoreNoteAuto(WristAngle.speakerShotFromSide, feeder, shooter, wrist, intake, robotState, log),

      //Resets pose, drives and intakes the close source-side note
      new DriveToAndIntakeNoteAuto(new Pose2d(1.1, 3.463, Rotation2d.fromDegrees(54)), new Pose2d(1.1, 4.6, Rotation2d.fromDegrees(-54)), TrajectoryType.driveToSourceCloseNote, driveTrain, feeder, shooter, wrist, intake, robotState, cache, alliance, log),

      //Drives back and sets wrist
      new DriveBackAndSetWristAuto(TrajectoryType.driveFromSourceNoteToSourceStart, WristAngle.speakerShotFromSpeaker, driveTrain, feeder, shooter, wrist, intake, robotState, cache, alliance, log),
      
      //Scores note
      new ShootPiece(false, shooter, feeder, wrist, robotState, log),

      //Prepares for teleop by driving and moving wrist to intaking position
      new ParallelCommandGroup(
        new WristSetAngle(WristAngle.lowerLimit, wrist, log),
        new ConditionalCommand(
            new DriveToPose(new Pose2d(7, 7.9, new Rotation2d(0)), driveTrain, log),
            new DriveToPose(new Pose2d(7, 0.3, new Rotation2d(0)), driveTrain, log),
          () -> alliance.getAlliance() == Alliance.Red
        )
      )
    );
  }
}
