// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Sequences;

import static edu.wpi.first.wpilibj2.command.Commands.*;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.FeederConstants;
import frc.robot.Constants.WristConstants.WristAngle;
import frc.robot.commands.FeederSetPercent;
import frc.robot.commands.WristSetAngle;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Wrist;
import frc.robot.utilities.FileLog;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class WristLowerSafe extends SequentialCommandGroup {

  /**
   * Lowers wrist, picking up any piece that may be in the belly pan.
   * @param angle target angle to move wrist to WristAngle (see Constants)
   * @param feeder feeder
   * @param wrist wrist
   * @param robotState state
   * @param log log
   */
  public WristLowerSafe(WristAngle angle, Feeder feeder, Wrist wrist, FileLog log) {

    addCommands(
      either(
        // Feeder currently does not have a piece
        parallel(
          sequence(

            // Pick up any piece that may be sitting in the belly pan
            new FeederSetPercent(FeederConstants.feederPercent, feeder, log),
            waitUntil(()->feeder.isPiecePresent()).withTimeout(4),
            either(
              sequence(   // Back off piece slightly from shooter wheels
                new FeederSetPercent(FeederConstants.feederBackPiecePercent, feeder, log),
                waitSeconds(FeederConstants.feederBackPieceTime),
                new FeederSetPercent(0.0, feeder, log)
              ),
              new FeederSetPercent(0, feeder, log),
              ()->feeder.isPiecePresent()
            )
          ).handleInterrupt( () -> { feeder.stopFeeder(); } ),  // If driver interrupts with another wrist/feeder command, then turn off the feeder motor.

          // Move wrist to desired angle in parallel with picking up any piece that may be sitting in the belly pan
          new WristSetAngle(angle, wrist, log)
        ),

        // Feeder already has a piece.  Just move wrist to desired angle
        new WristSetAngle(angle, wrist, log),

        () -> (!feeder.isPiecePresent() && angle.value <= WristAngle.clearBellyPanMinAngle.value)
      )
    );
  }
}
