// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Sequences;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Constants.CoordType;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.StopType;
import frc.robot.Constants.WristConstants.WristAngle;
import frc.robot.commands.DriveTrajectory;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Wrist;
import frc.robot.utilities.BCRRobotState;
import frc.robot.utilities.FileLog;
import frc.robot.utilities.TrajectoryCache;
import frc.robot.utilities.TrajectoryCache.TrajectoryType;
import frc.robot.utilities.AllianceSelection;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class DriveBackAndSetWristAuto extends ParallelCommandGroup {
  /** 
   * Drives to a given trajectory based on alliance, and primes the Shooter and Wrist for a speaker shot.
   * @param trajectory trajectory to follow (Changing with alliance)
   * @param wristAngle wrist target angle for shot, in degrees (+ = up, -  = down, 0 = horizontal)
   * @param drivetrain
   * @param feeder
   * @param shooter
   * @param wrist
   * @param intake
   * @param robotState
   * @param cache
   * @param alliance
   * @param log
  */
  public DriveBackAndSetWristAuto(TrajectoryType trajectory, WristAngle wristAngle, DriveTrain drivetrain, Feeder feeder, Shooter shooter, Wrist wrist, Intake intake, BCRRobotState robotState, TrajectoryCache cache, AllianceSelection alliance, FileLog log) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new DriveTrajectory(CoordType.kAbsolute, StopType.kBrake, cache.cache[trajectory.value], drivetrain, alliance, log),
      new SetShooterWristSpeakerAuto(wristAngle, ShooterConstants.shooterVelocityTop, ShooterConstants.shooterVelocityBottom, shooter, wrist, intake, feeder, robotState, log)
    );
  }
}
