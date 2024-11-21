// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autos;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.ShooterConstants;
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
public class AmpThreePieceCenter extends SequentialCommandGroup {
  /** Creates a new SourceTwoPieceShoot. */
  public AmpThreePieceCenter(Intake intake, Wrist wrist, Shooter shooter, DriveTrain driveTrain, Feeder feeder, BCRRobotState robotState, TrajectoryCache cache, AllianceSelection alliance, FileLog log) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());


    addCommands(
        // shoots
        new ScoreNoteAuto(WristAngle.speakerShotFromSide, feeder, shooter, wrist, intake, robotState, log),

        // leaves speaker from amp side to outside of notes and pick up first note
        new DriveToAndIntakeNoteAuto(new Pose2d(0.8, 1.6296, Rotation2d.fromDegrees(-60)), new Pose2d(0.8, 6.6, Rotation2d.fromDegrees(60)), TrajectoryType.driveAmpToFar2ndNote, driveTrain, feeder, shooter, wrist, intake, robotState, cache, alliance, log),

        new ConditionalCommand(
            new SequentialCommandGroup(
                // drive back to shoot note (Changed from a ParallelCommandGroup to a ParallelDeadlineGroup in command)
                new DriveBackAndSetWristAuto(TrajectoryType.driveNextCenterNotetoPodiumShot, WristAngle.ampFourPieceShot, driveTrain, feeder, shooter, wrist, intake, robotState, cache, alliance, log),
                
                // shoots piece
                new ShootPiece(ShooterConstants.shooterVelocityTop, ShooterConstants.shooterVelocityBottom, true, shooter, feeder, wrist, robotState, log),
                
                // drives back to grab middle center note
                new DriveToAndIntakeNoteAuto(TrajectoryType.drivePodiumShotToCenterNote, driveTrain, feeder, shooter, wrist, intake, robotState, cache, alliance, log)
            ),
            // We missed the note, go to the next note
            new DriveToAndIntakeNoteAuto(TrajectoryType.driveNextCenterNoteToCenterNote, driveTrain, feeder, shooter, wrist, intake, robotState, cache, alliance, log),
            
            () -> (feeder.isPiecePresent() == true || intake.getIntakeAmps() >= IntakeConstants.intakingPieceCurrentThreshold)
        ),
        new ConditionalCommand(
            new SequentialCommandGroup(
                // drive back under stage to shoot note
                new DriveBackAndSetWristAuto(TrajectoryType.driveCenterNotetoPodiumShot, WristAngle.endAmpFourcePieceShot, driveTrain, feeder, shooter, wrist, intake, robotState, cache, alliance, log),
                
                // shoots note
                new ShootPiece(ShooterConstants.shooterVelocityTop, ShooterConstants.shooterVelocityBottom, true, shooter, feeder, wrist, robotState, log)
            ),
            new SequentialCommandGroup(
                new WristSetAngle(WristAngle.lowerLimit, wrist, log)
            ),  
            () -> (feeder.isPiecePresent() == true || intake.getIntakeAmps() >= IntakeConstants.intakingPieceCurrentThreshold)
        )
    );
  }
}