// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix6.SignalLogger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveTrain;
import frc.robot.utilities.FileLog;


public class DrivePercentSpeed extends Command {
  private DriveTrain driveTrain;
  private FileLog log;

  // Parameters
  private double angleFacing, percentSpeed, maxDistance;
  private boolean fromShuffleboard;

  // Internal command variables
  private final Timer timer = new Timer();
  private Pose2d poseStart;
  private double curDistance;

  /**
   * Drives the robot straight at a fixed speed and stops after it travels a specified distance (or is cancelled).
   * Waits 1 second to align wheel facings prior to starting robot movement.
   * @param angleFacing Desired wheel facing relative to front of chassis in degrees, -180 to +180 (+=left, -=right, 0=facing front of robot)
   * @param percentSpeed Robot percent voltage, -1 to +1 (+ = in direction of angleFacing, - = opposite direction)
   * @param maxDistance Maximum distance for the robot to travel before ending this command, in meters
   * @param driveTrain
   * @param log
   */
   public DrivePercentSpeed(double angleFacing, double percentSpeed, double maxDistance, DriveTrain driveTrain, FileLog log) {
    this.driveTrain = driveTrain;
    this.log = log;
    this.angleFacing = angleFacing;
    this.percentSpeed= percentSpeed;
    this.maxDistance = maxDistance;
    fromShuffleboard = false;

    addRequirements(driveTrain);
  }

  /**
   * Drives the robot straight at a fixed speed and stops after it travels a specified distance (or is cancelled).
   * Waits 1 second to align wheel facings prior to starting robot movement.
   * Parameters angleFacing, percentSpeed, and maxDistance are chosen from Shuffleboard.
   * @param driveTrain
   * @param log
   */  public DrivePercentSpeed(DriveTrain driveTrain, FileLog log){
    this.driveTrain = driveTrain;
    this.log = log;
    fromShuffleboard = true;

    addRequirements(driveTrain);
    
    if(SmartDashboard.getNumber("DrivePercentSpeed maxDistance", -9999) == -9999) {
      SmartDashboard.putNumber("DrivePercentSpeed maxDistance", 0);
    }
    if(SmartDashboard.getNumber("DrivePercentSpeed angle", -9999) == -9999) {
      SmartDashboard.putNumber("DrivePercentSpeed angle", 0);
    }
    if(SmartDashboard.getNumber("DrivePercentSpeed percent", -9999) == -9999){
      SmartDashboard.putNumber("DrivePercentSpeed percent", .2);
    }
    SmartDashboard.putNumber("DrivePercentSpeed curDistance", 0.0);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.reset();
    timer.start();
    SignalLogger.start();

    poseStart = driveTrain.getPose();
    curDistance = 0.0;

    driveTrain.setDriveModeCoast(false);
    driveTrain.setVisionForOdometryState(false);      // Only use wheel encoders to track the robot for this command
    driveTrain.enableFastLogging(true);

    if(fromShuffleboard){
      maxDistance = SmartDashboard.getNumber("DrivePercentSpeed maxDistance", 0);
      angleFacing = SmartDashboard.getNumber("DrivePercentSpeed angle", 0);
      percentSpeed = SmartDashboard.getNumber("DrivePercentSpeed percent", 0);
    }

    log.writeLog(false, "DrivePercentSpeed", "Initialize", "Angle Facing Desired", angleFacing, "Percent Speed Deisred", percentSpeed,
                  "Max Distance", maxDistance);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //log.writeLog(false, "DrivePercentSpeed", "execute");
    curDistance = driveTrain.getPose().relativeTo(poseStart).getTranslation().getNorm();
    SmartDashboard.putNumber("DrivePercentSpeed curDistance", curDistance);
    
    driveTrain.setWheelFacings(angleFacing);
    if (timer.hasElapsed(1.0)) {
      driveTrain.setDriveMotorsOutput(percentSpeed);
    } else {
      driveTrain.setDriveMotorsOutput(0.0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveTrain.stopMotors();
    driveTrain.setVisionForOdometryState(true);
    driveTrain.enableFastLogging(false);
    timer.stop();
    SignalLogger.stop();

    log.writeLog(false, "DrivePercentSpeed", "End", "curDistance", curDistance);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return curDistance >= maxDistance;
  }
}
