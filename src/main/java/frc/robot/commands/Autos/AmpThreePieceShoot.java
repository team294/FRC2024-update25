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
public class AmpThreePieceShoot extends SequentialCommandGroup {
  /** Creates a new AmpThreePieceShoot. */
  public AmpThreePieceShoot(Intake intake, Wrist wrist, Shooter shooter, DriveTrain driveTrain, Feeder feeder, BCRRobotState robotState, TrajectoryCache cache, AllianceSelection alliance, FileLog log) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(

        // Sets wrist and shoots note
        new ScoreNoteAuto(WristAngle.speakerShotFromSpeaker, feeder, shooter, wrist, intake, robotState, log),

        // leaves speaker from amp side to outside of notes
        new DriveToAndIntakeNoteAuto(new Pose2d(0.8, 1.6296, Rotation2d.fromDegrees((-60))), new Pose2d(0.8, 6.6, Rotation2d.fromDegrees((60))), TrajectoryType.driveAmpToSecondFarCenter, driveTrain, feeder, shooter, wrist, intake, robotState, cache, alliance, log),

        // new VisionOdometryStateSet(true, driveTrain, log), // sending again incase auto init interferes with prior call

        // drive back to shoot note (Was parallel, changed to deadline group)
        new DriveBackAndSetWristAuto(TrajectoryType.driveNextCenterNotetoPodiumShot, WristAngle.ampFourPieceShot, driveTrain, feeder, shooter, wrist, intake, robotState, cache, alliance, log),
        
        // shoots note
        new ShootPiece(ShooterConstants.shooterVelocityTop, ShooterConstants.shooterVelocityBottom, true, shooter, feeder, wrist, robotState, log),
        
        // drives back to grab middle center note
        new DriveToAndIntakeNoteAuto(TrajectoryType.drivePodiumShotToCenterNote, driveTrain, feeder, shooter, wrist, intake, robotState, cache, alliance, log),
       
        // drive back under stage to shoot note
        new DriveBackAndSetWristAuto(TrajectoryType.driveCenterNotetoPodiumShot, WristAngle.sourceThreePieceShot, driveTrain, feeder, shooter, wrist, intake, robotState, cache, alliance, log),
        
        // shoots note
        new ShootPiece(ShooterConstants.shooterVelocityTop, ShooterConstants.shooterVelocityBottom, true, shooter, feeder, wrist, robotState, log)
        
              
    );
  }
}
