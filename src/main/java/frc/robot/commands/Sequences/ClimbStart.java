// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Sequences;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Constants.LEDConstants.LEDSegmentRange;
import frc.robot.Constants.WristConstants.WristAngle;
import frc.robot.commands.CANdleRainbowAnimation;
import frc.robot.commands.WristSetAngle;
import frc.robot.subsystems.LED;
import frc.robot.subsystems.Wrist;
import frc.robot.utilities.FileLog;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ClimbStart extends ParallelCommandGroup {
  /** Raises wrist to prepare for climb
   * @param wrist
   * @param log
   * @param leds
  */
  public ClimbStart(Wrist wrist, FileLog log, LED led) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new WristSetAngle(WristAngle.climbStart, wrist, log),
      new CANdleRainbowAnimation(led, LEDSegmentRange.StripHorizontal)
    );
  }
}
