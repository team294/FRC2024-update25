// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autos;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.WristConstants.WristAngle;
import frc.robot.commands.WristSetAngle;
import frc.robot.commands.Sequences.ResetPoseAndDriveToPosAuto;
import frc.robot.commands.Sequences.ScoreNoteAuto;
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
public class SourceWallMobilityAuto extends SequentialCommandGroup {
  /** Creates a new SourceWallMobilityAuto. */
  public SourceWallMobilityAuto(Intake intake, Wrist wrist, Shooter shooter, DriveTrain driveTrain, Feeder feeder, BCRRobotState robotState, TrajectoryCache cache, AllianceSelection alliance, FileLog log) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      //Sets wrist and scores note
      new ScoreNoteAuto(WristAngle.speakerShotFromSpeaker, feeder, shooter, wrist, intake, robotState, log),

      new ParallelCommandGroup(
        //Resets pose and drive to score mobility points
        new ResetPoseAndDriveToPosAuto(new Pose2d(0.8, 3.7296, Rotation2d.fromDegrees(60)), new Pose2d(0.8, 4.5, Rotation2d.fromDegrees(-60)), TrajectoryType.driveFromSourceToWallMobility, driveTrain, cache, alliance, log),
        
        //Prepares wrist to intake in teleop
        new WristSetAngle(WristAngle.lowerLimit, wrist, log)
      )
    );
  }
}
