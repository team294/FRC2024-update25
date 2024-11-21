// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Wrist;
import frc.robot.utilities.FileLog;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class WristNudgeAngle extends InstantCommand {
  private double deltaDegrees;
  private Wrist wrist;
  private FileLog log;
  private boolean fromShuffleboard;

  /**
   * Adjust the current calibration degrees of the wrist by a small amount
   * @param deltaDegrees the number of degrees to move up/down.  + = down, - = up
   * @param wrist
   * @param log
   */
  public WristNudgeAngle(double deltaDegrees, Wrist wrist, FileLog log) {
    this.deltaDegrees = deltaDegrees;
    this.wrist = wrist;
    this.log = log;
    fromShuffleboard = false;

    addRequirements(wrist);
  }

  /**
   * Adjust the current calibration degrees of the wrist by a small amount (from SmartDashboard)
   * @param deltaDegrees the number of degrees to move up/down (Wrist Nudge Delta Degrees from SmartDashboard).  + = down, - = up
   * @param wrist
   * @param log
   */
  public WristNudgeAngle(Wrist wrist, FileLog log) {
    this.wrist = wrist;
    this.log = log;
    fromShuffleboard = true;

    if (SmartDashboard.getNumber("Wrist Nudge Delta Degrees", -9999) == -9999) {
      SmartDashboard.putNumber("Wrist Nudge Delta Degrees", 0);
    }

    addRequirements(wrist);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (fromShuffleboard) {
      deltaDegrees = SmartDashboard.getNumber("Wrist Nudge Delta Degrees", 0);
    }
    wrist.nudgeWristAngle(deltaDegrees);
    log.writeLog(false, "WristNudgeAngle", "Initialize", "Target", deltaDegrees);
  }
}
