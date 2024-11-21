/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.WristConstants.WristAngle;
import frc.robot.subsystems.Wrist;
import frc.robot.utilities.FileLog;

public class WristSetAngle extends Command {

  private double angle;
  private final Wrist wrist;
  private final FileLog log;
  private final boolean fromShuffleboard;
  private final boolean isAmp;

  /**
   * Moves wrist to target angle.  Command ends when wrist is within 5 degrees of the target position.
   * <p> This command does nothing and immediately returns if the wrist is not calibrated.
   * @param angle target angle in degrees.  (0 = horizontal in front of robot, + = up, - = down)
   */
  public WristSetAngle(double angle, Wrist wrist, FileLog log) {
    this.angle = angle;
    this.wrist = wrist;
    this.log = log;
    fromShuffleboard = false;
    isAmp = false;

    addRequirements(wrist);
  }

  /**
   * Moves wrist to target position.  Command ends when wrist is within 5 degrees of the target position.
   * <p> This command does nothing and immediately returns if the wrist is not calibrated.
   * @param position WristAngle (see Constants)
   */
  public WristSetAngle(WristAngle pos, Wrist wrist, FileLog log) {
    angle = pos.value;
    this.wrist = wrist;
    this.log = log;
    fromShuffleboard = false;
    isAmp = false;

    addRequirements(wrist);
  }

  /**
   * Moves wrist to target angle from Shuffleboard.  Command ends when wrist is within 5 degrees of the target position.
   * <p> This command does nothing and immediately returns if the wrist is not calibrated.
   */
  public WristSetAngle(Wrist wrist, FileLog log) {
    this.wrist = wrist;
    this.log = log;
    fromShuffleboard = true;
    isAmp = false;

    if(SmartDashboard.getNumber("Wrist Angle to set", -9999) == -9999) {
      SmartDashboard.putNumber("Wrist Angle to set", 0);
    }
    addRequirements(wrist);
  }

   /**
    * Moves wrist to target angle from Shuffleboard.  Command ends when wrist is within 5 degrees of the target position.
    * <p> This command does nothing and immediately returns if the wrist is not calibrated.
    * @param isAmp True = Move wrist to amp angle with nudge offset.  False = Move wrist to amp angle without nudge offset.  
    * @param wrist
    * @param log
    */
  public WristSetAngle(boolean isAmp, Wrist wrist, FileLog log) {
    this.isAmp = isAmp;
    this.wrist = wrist;
    this.log = log;
    fromShuffleboard = false;
    angle = WristAngle.ampShot.value;

    addRequirements(wrist);
  }

  // Called just before this Command runs the first time
  @Override
  public void initialize() {
    if(fromShuffleboard){
      angle = SmartDashboard.getNumber("Wrist Angle to set", 0);
    }
    if(isAmp){
      angle = WristAngle.ampShot.value;
      angle += wrist.getAmpOffSet();
    }
    wrist.setWristAngle(angle);
    log.writeLog(false, "WristSetAngle", "Initialize", "Target", angle);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  public void execute() {
    wrist.updateWristLog(false);
  }

  // Called once after isFinished returns true
  @Override
  public void end(boolean interrupted) {
    log.writeLog(false, "WristSetAngle", "End", "Target", angle, "Current angle", wrist.getWristAngle());
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  public boolean isFinished() {
    return !wrist.isEncoderCalibrated() || Math.abs(wrist.getWristAngle() - wrist.getCurrentWristTarget()) < 5.0; // tolerance of 5 degrees
  }
}