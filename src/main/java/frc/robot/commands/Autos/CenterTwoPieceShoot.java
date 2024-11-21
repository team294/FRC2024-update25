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
public class CenterTwoPieceShoot extends SequentialCommandGroup {
  /** Creates a new CenterTwoPieceShoot. */
  public CenterTwoPieceShoot(Intake intake, Wrist wrist, Shooter shooter, DriveTrain driveTrain, Feeder feeder, BCRRobotState robotState, TrajectoryCache cache, AllianceSelection alliance, FileLog log) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      //Scores note
      new ScoreNoteAuto(WristAngle.speakerShotFromSpeaker, feeder, shooter, wrist, intake, robotState, log),

      //Resets pose, drive to and intakes center close note
      new DriveToAndIntakeNoteAuto(new Pose2d(1.3, 2.663, Rotation2d.fromDegrees(0)), new Pose2d(1.3, 5.57, Rotation2d.fromDegrees(0)), TrajectoryType.driveToCenterCloseNote, driveTrain, feeder, shooter, wrist, intake, robotState, cache, alliance, log),
      
      //Drive back to score note
      new DriveBackAndSetWristAuto(TrajectoryType.driveFromCenterNoteToCenterStart, WristAngle.speakerShotFromSpeaker, driveTrain, feeder, shooter, wrist, intake, robotState, cache, alliance, log),
      
      //Scores note
      new ShootPiece(ShooterConstants.shooterVelocityTop, ShooterConstants.shooterVelocityBottom, true, shooter, feeder, wrist, robotState, log)
    );
  }
}
