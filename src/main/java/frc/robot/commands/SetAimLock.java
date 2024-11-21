// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveTrain;
import frc.robot.utilities.FileLog;

public class SetAimLock extends Command {

  private final DriveTrain driveTrain;
  private final FileLog log;

  private final boolean state;

  /**
   * Turns on or off robot "aim rotation lock" on the target (speaker or long pass target)
   * for DriveWithJoysticksAdvance.
   * @param driveTrain
   * @param state true = aim rotation lock on, false = aim rotation lock off
   * @param log
   */
  public SetAimLock(DriveTrain driveTrain, boolean state, FileLog log) {
    this.driveTrain = driveTrain;
    this.state = state;
    this.log = log;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    driveTrain.setAimLock(state);
    log.writeLog(false, "SetAimLock", "Initialize", "AimLock", state);
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
}
