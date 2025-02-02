// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Sequences;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.*;
import frc.robot.commands.DriveStraight;
import frc.robot.commands.DriveToPose;
import frc.robot.subsystems.DriveTrain;
import frc.robot.utilities.Field;
import frc.robot.utilities.FileLog;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class DriveToClosestReefPosition extends SequentialCommandGroup {
  /** Drives to the closest reef scoring position with some offset to allow rotation, before driving up against the reef
   * 
  */
  public DriveToClosestReefPosition(DriveTrain drivetrain, Field field, FileLog log) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      //Drives to the nearest scoring position (which is on the wall), with an offset of half the robot's diameter plus 5cm
      new DriveToPose(() -> (field.getNearestReefScoringPositionWithOffset(drivetrain.getPose(), new Transform2d((-RobotDimensions.robotDiagonal/2) - 0.05, 0, new Rotation2d(0)))), drivetrain, log),
      
      //Drives the remaining distance into the reef
      //TODO Find reasonable swerve constants to drive the remaining distance (~0.25 meters? [Measurement was calculated using 2025 ETU])
      new DriveStraight(((RobotDimensions.robotDiagonal-RobotDimensions.robotWidth)/2) + 0.05, false, 0, 1.0, SwerveConstants.kNominalAccelerationMetersPerSecondSquare, true, false, drivetrain, log).withTimeout(0.5)
    );
  }
}
