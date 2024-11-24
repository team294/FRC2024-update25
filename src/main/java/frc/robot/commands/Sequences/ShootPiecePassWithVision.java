// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Sequences;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants.FeederConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.WristConstants;
import frc.robot.commands.*;
import frc.robot.commands.ShooterSetVelocity.VelocityType;
import frc.robot.subsystems.*;
import frc.robot.utilities.AllianceSelection;
import frc.robot.utilities.BCRRobotState;
import frc.robot.utilities.FileLog;
import frc.robot.utilities.BCRRobotState.ShotMode;

public class ShootPiecePassWithVision extends SequentialCommandGroup {

  public interface VelocityGetter {
    /**
     * When an object implementing interface {@code Runnable} is used
     * to create a thread, starting the thread causes the object's
     * {@code run} method to be called in that separately executing
     * thread.
     * <p>
     * The general contract of the method {@code run} is that it may
     * take any action whatsoever.
     *
     */
    public abstract double get();
  }

  /**
   * Shoots a piece using variable shooter velocity.
   * <p><b> NOTE: </b> This only works for ShotMode = VISION_FAR_PASS or VISION_MID_PASS!!!!!!!
   * @param waitForSpinDown true = wait for shooter motors to stop before returning.  False = return immediately after shooting, with shooter motors set to slow reverse speed.
   * @param allianceSelection
   * @param driveTrain
   * @param shooter
   * @param feeder
   * @param wrist (not a required subsystem -- only reads the arm angle)
   * @param robotState
   * @param log
   */
  public ShootPiecePassWithVision(boolean waitForSpinDown, AllianceSelection allianceSelection, DriveTrain driveTrain, Shooter shooter, Feeder feeder, Wrist wrist, BCRRobotState robotState, FileLog log) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      // Make sure that shooter is at speed and wrist is at target angle (if it is calibrated)
      new ParallelCommandGroup(
        new RobotStateSet(BCRRobotState.State.SHOOTING, robotState, log),
        new ShooterSetVelocity(() -> {
          double x = driveTrain.getPose().getX() - (robotState.getShotMode() == ShotMode.VISION_FAR_PASS ? allianceSelection.getFarPassXPos() : allianceSelection.getMidPassXPos());
          double y = driveTrain.getPose().getY() - (robotState.getShotMode() == ShotMode.VISION_FAR_PASS ? allianceSelection.getFarPassYPos() : allianceSelection.getMidPassYPos());
          // distance from center of robot to shooter
          double dist = Math.sqrt(x*x+y*y);

          return (290.0 * dist) + 94.4051;   // F3:  Increased multipler from 280 to 290
        }, VelocityType.waitForVelocity, shooter, log),
        new WaitUntilCommand( () -> !wrist.isEncoderCalibrated() || (Math.abs(wrist.getCurrentWristTarget() - wrist.getWristAngle()) < WristConstants.wristShootTolerance) )
      ).withTimeout(1.5),

      // Shoot
      new FeederSetPercent(FeederConstants.feederPercent, feeder, log),
      new ConditionalCommand(
        // If we have a piece, then shoot 0.1 sec after the piece leaves the feeder sensor
        new SequentialCommandGroup(
          new WaitUntilCommand(()-> !feeder.isPiecePresent()).withTimeout(.4),
          new WaitCommand(.1)
        ),
        // If we don't have a piece (or the piece sensor is broken), then shoot for 0.5 sec
        new WaitCommand(0.5),
        () -> feeder.isPiecePresent()
      ),

      // After shot, reverse shooter and turn off feeder 
      new ParallelCommandGroup(
        new ShooterSetPercent(ShooterConstants.shooterPercentStopQuickly, shooter, log),
        new FeederSetPercent(0, feeder, log),
        new RobotStateSetIdle(robotState, feeder, log)
      )
    );

    // If we are waiting for spin down, then turn off feeder after 0.5 sec
    if (waitForSpinDown) {
      addCommands(
        new WaitCommand(ShooterConstants.shooterSpinDownSeconds),
        new ShooterFeederStop(shooter, feeder, log)
      );
    }
  }

}
