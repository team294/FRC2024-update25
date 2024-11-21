// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autos;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
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
public class SourceFourNoteCenter extends SequentialCommandGroup {
  /** Creates a new SourceTwoPieceShoot. */
  public SourceFourNoteCenter(Intake intake, Wrist wrist, Shooter shooter, DriveTrain driveTrain, Feeder feeder, BCRRobotState robotState, TrajectoryCache cache, AllianceSelection alliance, FileLog log) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());

    addCommands(
        // shoots
        new ScoreNoteAuto(WristAngle.speakerShotFromSide, feeder, shooter, wrist, intake, robotState, log),
    
        // leaves speaker from source side to outside of notes
        new ResetPoseAndDriveToPosAuto(new Pose2d(0.8, 3.73, Rotation2d.fromDegrees(54)), new Pose2d(0.8, 4.5, Rotation2d.fromDegrees(-54)), TrajectoryType.driveSourceOutsideNotes, driveTrain, cache, alliance, log),

        // goes around stage intake up note left of middle center note
        new DriveToAndIntakeNoteAuto(TrajectoryType.driveOutsideStageLeftCenterNote, driveTrain, feeder, shooter, wrist, intake, robotState, cache, alliance, log),

        // drive back under stage to shoot note
        new DriveBackAndSetWristAuto(TrajectoryType.driveLeftCenterNotetoOutsideStage, WristAngle.sourceThreePieceShot, driveTrain, feeder, shooter, wrist, intake, robotState, cache, alliance, log),

        // shoots in speaker   
        new ShootPiece(ShooterConstants.shooterVelocityTop, ShooterConstants.shooterVelocityBottom, true, shooter, feeder, wrist, robotState, log),
        
        // drives back through under stage to grab middle center note
        new DriveToAndIntakeNoteAuto(TrajectoryType.driveSourceOutsideNotestoCenterNote, driveTrain, feeder, shooter, wrist, intake, robotState, cache, alliance, log),
       
        // drives back to shoot in speaker
        new DriveBackAndSetWristAuto(TrajectoryType.driveCenterNotetoOutsideStage, WristAngle.sourceThreePieceShot, driveTrain, feeder, shooter, wrist, intake, robotState, cache, alliance, log),
        
        // shoots note
        new ShootPiece(ShooterConstants.shooterVelocityTop, ShooterConstants.shooterVelocityBottom, true, shooter, feeder, wrist, robotState, log),
        
        // drives back through under stage to grab right of middle center note
        new DriveToAndIntakeNoteAuto(TrajectoryType.drivePodiumShotToCenterRightNote, driveTrain, feeder, shooter, wrist, intake, robotState, cache, alliance, log),
        
        // drives back to shoot in speaker
        new DriveBackAndSetWristAuto(TrajectoryType.driveCenterRightNoteToPodiumShot, WristAngle.sourceThreePieceShot, driveTrain, feeder, shooter, wrist, intake, robotState, cache, alliance, log),
        
        // shoots note
        new ShootPiece(ShooterConstants.shooterVelocityTop, ShooterConstants.shooterVelocityBottom, true, shooter, feeder, wrist, robotState, log)
    );
  }
}