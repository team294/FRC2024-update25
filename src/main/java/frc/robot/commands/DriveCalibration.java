// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix6.SignalLogger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveTrain;
import frc.robot.utilities.FileLog;

public class DriveCalibration extends Command {

  private DriveTrain driveTrain;
  private FileLog log;
  private final double alignTime = 1.0;     // Align wheels for 1.0 second before starting ramp.
  private double angleFacing, percentOutput, maxPercentOutput, endTime, rampRate;
  private final Timer timer = new Timer();

  /**
   * Drives the robot straight with a ramp velocity, starting at 0 speed.
   * Waits 1 second to align wheel facings prior to starting robot movement, then ramps up speed for "rampTime" seconds.
   * @param angleFacing Desired wheel facing relative to front of chassis in degrees, -180 to +180 (+=left, -=right, 0=facing front of robot)
   * @param maxPercentOutput % output (between 0 and 1)
   * @param rampTime  ramp up time in seconds
   * @param rampRate Ramp rate in pctOut/second 
   * @param driveTrain
   * @param log
   */
  public DriveCalibration(double angleFacing, double maxPercentOutput, double rampTime, double rampRate, DriveTrain driveTrain, FileLog log) {
    this.driveTrain = driveTrain;
    this.log = log;
    this.angleFacing = angleFacing;
    this.maxPercentOutput = maxPercentOutput;
    this.endTime = rampTime + alignTime;
    this.rampRate = rampRate;

    addRequirements(driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.reset();
    timer.start();
    SignalLogger.start();

    driveTrain.setDriveModeCoast(false);
    driveTrain.setVisionForOdometryState(false);      // Only use wheel encoders to track the robot for this command
    driveTrain.enableFastLogging(true);

    log.writeLog(false, "DriveCalibration", "Initialize", "maxPctOut", maxPercentOutput, "rampTime", endTime, "rampRate", rampRate);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double currTime = timer.get();

    driveTrain.setWheelFacings(angleFacing);

    if (timer.hasElapsed(alignTime)) {
      percentOutput = MathUtil.clamp((currTime-alignTime)*rampRate, -maxPercentOutput, maxPercentOutput);
      driveTrain.setDriveMotorsOutput(percentOutput);
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

    log.writeLog(false, "DriveCalibration", "End");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return timer.hasElapsed(endTime);
  }
}
