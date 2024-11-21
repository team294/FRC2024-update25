// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Sequences;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.WristConstants.WristAngle;
import frc.robot.Constants.FeederConstants;
import frc.robot.commands.*;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Wrist;
import frc.robot.utilities.BCRRobotState;
import frc.robot.subsystems.Feeder;
import frc.robot.utilities.FileLog;

public class IntakePiece extends SequentialCommandGroup {

  /**
   * Moves the wrist down, then turns on the intake and feeder motors, and sets the robot state
   * to INTAKE_TO_FEEDER (but only if we don't have a piece).
   * <p><b>Note:</b> This sequence does not stop intaking!  This requires a 
   * trigger to turn off intaking when a piece gets to the feeder, or the
   * driver can manually turn off intaking.
   * @param intake
   * @param feeder
   * @param wrist
   * @param shooter
   * @param robotState
   * @param log
   */
  public IntakePiece(Intake intake, Feeder feeder, Wrist wrist, Shooter shooter, BCRRobotState robotState, FileLog log) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());

    addCommands(
      new ConditionalCommand(
        new ParallelCommandGroup(
          new WristSetAngle(WristAngle.lowerLimit, wrist, log),
          new RobotStateSet(BCRRobotState.State.INTAKING, robotState, log),
          new SequentialCommandGroup(
            new WaitUntilCommand(() -> wrist.getWristAngle() < WristAngle.intakeLimit.value).withTimeout(1.5),
            new IntakeSetPercent(IntakeConstants.intakePercent,IntakeConstants.centeringPercent, intake, log)
          ),
          new FeederSetPercent(FeederConstants.feederPercent, feeder, log),
          // Spin down the shooter if needed
          new ConditionalCommand(
            new SequentialCommandGroup(
              new ShooterSetPercent(ShooterConstants.shooterPercentStopQuickly, shooter, log),
              new WaitCommand(ShooterConstants.shooterSpinDownSeconds),
              new ShooterSetPercent(0.0, shooter, log) // Stop the shooter, not the feeder
            ).handleInterrupt(() -> { shooter.stopMotors(); }),
            new FileLogWrite(false, false, "IntakePiece", "Shooter already stopped", log),
            () -> (shooter.getBottomShooterVelocity() > 200.0)
          )
        ),
        new FileLogWrite(false, false, "IntakePiece", "Already has piece", log),
        () -> (!feeder.isPiecePresent())
      )
    );
  }
}
