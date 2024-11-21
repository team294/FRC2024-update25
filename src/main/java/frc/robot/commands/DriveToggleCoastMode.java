// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveTrain;
import frc.robot.utilities.FileLog;

public class DriveToggleCoastMode extends Command {
    
    private final DriveTrain driveTrain;
    private final FileLog log;

/**
 * Toggle drivetraing between coast and break drive modes
 * @param driveTrain
 * @param log
 */
public DriveToggleCoastMode(DriveTrain driveTrain, FileLog log) {
    this.driveTrain = driveTrain;
    this.log = log;

    addRequirements(driveTrain);
}

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if(driveTrain.isDriveModeCoast()){   //if drive mode is already in coast, switch to brake
      driveTrain.setDriveModeCoast(false);
    } else {
      driveTrain.setDriveModeCoast(true); //Otherwise, switch to brake since drive mode already in coast
    }

    log.writeLog(true, "DriveToggleCoastMode", "Drive Mode Toggled from Shuffleboard", "Coast Mode", driveTrain.isDriveModeCoast());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }

  /**
   * Whether the given command should run when the robot is disabled. Override to return true if the
   * command should run when disabled.
   * @return whether the command should run when the robot is disabled
   */
  @Override
  public boolean runsWhenDisabled() {
    return true;
  } 
}
