// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.DriveTrain;
import frc.robot.utilities.FileLog;

public class VisionOdometryStateSet extends InstantCommand {
  private final boolean enabled;
  private final DriveTrain driveTrain;
  private final FileLog log;

  /**
   * Turns on or off vision updates for odometry.
   * @param enabled true = uses vision for odometry, false = does not use vision for odometry   
   * @param driveTrain
   * @param log log
   */
  public VisionOdometryStateSet(boolean enabled, DriveTrain driveTrain, FileLog log) {
    this.enabled = enabled;
    this.driveTrain = driveTrain;
    this.log = log;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    driveTrain.setVisionForOdometryState(enabled);
    log.writeLog(true, "VisionOdometryStateSet", "Initialize", "Enabled", (enabled) ? "yes" : "np");
  }
  
  // Allows for running while robot is disabled
  @Override
  public boolean runsWhenDisabled() {
    return true;
  }
}
  