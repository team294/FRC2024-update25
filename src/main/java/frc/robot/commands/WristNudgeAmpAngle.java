// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Wrist;
import frc.robot.utilities.FileLog;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class WristNudgeAmpAngle extends InstantCommand {
  private double deltaDegrees;
  private Wrist wrist;
  private FileLog log;

  /**
   * Adjust the current amp shooting angle of the wrist by a small amount
   * @param deltaDegrees the number of degrees to move up/down
   * @param wrist
   * @param log
   */
  public WristNudgeAmpAngle(double deltaDegrees, Wrist wrist, FileLog log) {
    this.deltaDegrees = deltaDegrees;
    this.wrist = wrist;
    this.log = log;

    addRequirements(wrist);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
    wrist.nudgeAmpAngle(deltaDegrees);
    log.writeLog(false, "WristNudgeAmpAngle", "Initialize", "Target", deltaDegrees);
  }
}
