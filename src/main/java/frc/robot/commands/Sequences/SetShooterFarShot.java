// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Sequences;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.WristConstants.WristAngle;
import frc.robot.commands.ShotModeSet;
import frc.robot.commands.IntakeStop;
import frc.robot.commands.RobotStateSetIdle;
import frc.robot.commands.ShooterSetVelocity;
import frc.robot.commands.WristSetAngle;
import frc.robot.commands.ShooterSetVelocity.VelocityType;
import frc.robot.subsystems.*;
import frc.robot.utilities.*;
import frc.robot.utilities.BCRRobotState.ShotMode;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SetShooterFarShot extends SequentialCommandGroup {

  /**
   * Sets Shooter and Wrist to prime for a Far shot (lobbing note towards alliance partner).  
   * Also stops the intake and sets the robot state.
   * @param angle wrist target angle for shot, in degrees (+ = up, -  = down, 0 = horizontal)
   * @param velocityTop top shooter wheel velocity, in rpm  (+ = shoot forward, - = backwards)
   * @param velocityBottom bottom shooter wheel velocity, in rpm  (+ = shoot forward, - = backwards)
   * @param shooter
   * @param wrist
   * @param intake
   * @param feeder
   * @param robotState
   * @param log
   */
  public SetShooterFarShot(WristAngle angle, double velocityTop, double velocityBottom, 
    Shooter shooter, Wrist wrist, Intake intake, Feeder feeder, ShotMode shotMode, BCRRobotState robotState, FileLog log) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        new ParallelCommandGroup(
          new IntakeStop(intake, log),
          new WristSetAngle(angle, wrist, log),
          new ShooterSetVelocity(velocityTop, velocityBottom, VelocityType.waitForVelocity, shooter, log),
          new ShotModeSet(shotMode, robotState, log),
          new RobotStateSetIdle(robotState, feeder, log)
        )
      );
  }
}

