// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.VisionConstants.PhotonVisionConstants;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.PhotonCameraWrapper;
import frc.robot.utilities.FileLog;


public class DisplayPosition extends Command {
  private DriveTrain driveTrain;
  private FileLog log;
  private int logRotationKey;
  private PhotonPoseEstimator poseEstimator;

  // Internal command variables
  private PhotonCameraWrapper camera;

  /**
   * Drives the robot straight at a fixed speed and stops after it travels a specified distance (or is cancelled).
   * Parameters angleFacing, percentSpeed, and maxDistance are chosen from Shuffleboard.
   * @param driveTrain
   * @param log
   */  public DisplayPosition(DriveTrain driveTrain, FileLog log){
    this.driveTrain = driveTrain;
    this.log = log;
    this.logRotationKey = log.allocateLogRotation();
    this.poseEstimator = new PhotonPoseEstimator(AprilTagFieldLayout.loadFromResource(AprilTagFields.k2024Crescendo.m_resourceFile), PoseStrategy.CLOSEST_TO_REFERENCE_POSE, PhotonVisionConstants.robotToCamBack);
    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    

    //log.writeLog(false);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //log.writeLog(false, "DrivePercentSpeed", "execute");
    
    SmartDashboard.putNumberArray("Zander Estimated Pose", new Double[poseEstimator.update(Rotation2d.fromDegrees(getGyroRotation()), getModulePositions()).getReferencePose()]);
    }
      //poseEstimator.update(Rotation2d.fromDegrees(poseEstimator.update(this.driveTrain.getGyroRotation()), this.driveTrain.getModulePositions())
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //log.writeLog();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // return timer.hasElapsed(maxDistance);
    return true;
  }
}
