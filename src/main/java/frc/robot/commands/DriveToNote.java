/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.SwerveConstants;
import frc.robot.Constants.VisionConstants.PhotonVisionConstants;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Feeder;
import frc.robot.utilities.*;

/**
 * Command to drive to a note using the JeVois camera and the drive train
 */
public class DriveToNote extends Command {
  private final DriveTrain driveTrain;
  private final Feeder feeder; 
  private final PIDController fwdRateController;
  private final PIDController leftRateController;
  private final PIDController turnRateController;
  private final FileLog log;
  private int toleranceCount;
  private int logRotationKey;

  private PhotonPipelineResult latestResult;
  
  private double fwdVelocity, leftVelocity, turnRate;
  // private double lastFwdPercent, lastTime, curTime;

  // private final double maxFwdRateChange = 2.0;
  // private final double maxRevRateChange = -1.4;

  /**
   * @param driveTrain drive train subsystem to use
   * @param log filelog to use
   */
  public DriveToNote(Feeder feeder, DriveTrain driveTrain, FileLog log) {
    this.feeder = feeder;
    this.driveTrain = driveTrain;
    this.log = log;
    fwdRateController = new PIDController(0.13, 0.065, 0);
    leftRateController = new PIDController(0.0, 0, 0);
    turnRateController = new PIDController(0.15, 0, 0);
    fwdRateController.setSetpoint(PhotonVisionConstants.pitchSetpoint); // setpoint is the bottom of the screen
    leftRateController.setSetpoint(PhotonVisionConstants.yawSetpoint); // setpoint is the middle line on the screen
    turnRateController.setSetpoint(PhotonVisionConstants.yawSetpoint); // setpoint is the middle line on the screen

    logRotationKey = log.allocateLogRotation();

    addRequirements(driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    driveTrain.setDriveModeCoast(false);
    
    toleranceCount = 0;

    // lastFwdPercent = 0;
    // lastTime = System.currentTimeMillis() / 1000.0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // curTime = System.currentTimeMillis() / 1000.0;
    
    if (!driveTrain.getNoteCamera().hasInit()) return;

    PhotonPipelineResult result = driveTrain.getLatestResult();
    if (result != null) {
      latestResult = result;
    }
    // Possibility latestResult will not be defined if something
    // else gets the latest unread results in the same code cycle
    if (!latestResult.hasTargets() || result == null) {
      if(log.isMyLogRotation(logRotationKey)) {
        log.writeLog(false, "DriveToNote", "No targets captured");
      }
      toleranceCount++;
      // stop if can't see piece
      // driveTrain.drive(0, 0, 0, false, false);
      return;
    }


    toleranceCount = 0;
    PhotonTrackedTarget bestTarget = latestResult.getBestTarget();
    

    fwdVelocity = -fwdRateController.calculate(bestTarget.getPitch());
    leftVelocity = leftRateController.calculate(bestTarget.getYaw());
    turnRate = turnRateController.calculate(bestTarget.getYaw());

    fwdVelocity = MathUtil.clamp(fwdVelocity, -SwerveConstants.kMaxSpeedMetersPerSecond, SwerveConstants.kMaxSpeedMetersPerSecond);
    leftVelocity = MathUtil.clamp(leftVelocity, -SwerveConstants.kMaxSpeedMetersPerSecond, SwerveConstants.kMaxSpeedMetersPerSecond);
    turnRate = MathUtil.clamp(turnRate, -SwerveConstants.kMaxTurningRadiansPerSecond, SwerveConstants.kMaxTurningRadiansPerSecond);

    if(log.isMyLogRotation(logRotationKey)) {
      System.out.println(bestTarget.getYaw());
      log.writeLog(false, "DriveToNote", "Joystick", "Fwd", fwdVelocity, "Left", leftVelocity, "Turn", turnRate);
    }
    
    // double fwdRateChange = (fwdPercent - lastFwdPercent) / (curTime - lastTime);
    // if (fwdRateChange > maxFwdRateChange) {
    //   fwdPercent = lastFwdPercent + (curTime - lastTime)*maxFwdRateChange;
    // } else if (fwdRateChange < maxRevRateChange) {
    //   fwdPercent = lastFwdPercent +(curTime - lastTime)*maxRevRateChange;

    // }

    SmartDashboard.putNumber("DriveToNote fwd", fwdVelocity);
    SmartDashboard.putNumber("DriveToNote left", leftVelocity);
    SmartDashboard.putNumber("DriveToNote turnRate", turnRate);
    SmartDashboard.putNumber("DriveToNote BestTargetPitch", bestTarget.getPitch());
    SmartDashboard.putNumber("DriveToNote BestTargetYaw", bestTarget.getYaw());
    
    driveTrain.drive(fwdVelocity, leftVelocity, turnRate, false, false);

    // lastFwdPercent = fwdPercent;
    // lastTime = curTime;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return !driveTrain.getNoteCamera().hasInit() || feeder.isPiecePresent() 
      || toleranceCount > 4;
  }
}