// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Sequences;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.*;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Feeder;
import frc.robot.utilities.FileLog;
import frc.robot.utilities.BCRRobotState;

public class StopIntakeFeederShooter extends ParallelCommandGroup {

  /**
   * Stops the intake, feeder, and shooter motors.
   * Resets the robot state to IDLE
   * @param intake intake object
   * @param shooter shooter object
   * @param feeder feeder object
   * @param robotState Object with current robot state
   * @param log log
   */
  public StopIntakeFeederShooter(Intake intake, Shooter shooter, Feeder feeder, BCRRobotState robotState, FileLog log) {
    addCommands(
      new IntakeStop(intake, log),
      new ShooterFeederStop(shooter, feeder, log),
      new RobotStateSetIdle(robotState, feeder, log)
    );
  }
}
