// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;
import frc.robot.utilities.FileLog;

public class ShooterSetPercent extends Command {
  private double percent = 0.0;
  private final FileLog log;
  private final Shooter shooter;
  private boolean fromShuffleboard;

  /** Creates a new ShooterSetPercent. */
  public ShooterSetPercent(double percent, Shooter shooter, FileLog log) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.percent = percent;
    this.log = log;
    this.shooter = shooter;
    this.fromShuffleboard = false;
    addRequirements(shooter);
  }

  public ShooterSetPercent(Shooter shooter, FileLog log) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.percent = 0.0;
    this.log = log;
    this.shooter = shooter;
    this.fromShuffleboard = true;
    addRequirements(shooter);

    if(SmartDashboard.getNumber("Shooter Percent", -9999.9) == -9999.9) {
      SmartDashboard.putNumber("Shooter Percent", 0);
    }
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (fromShuffleboard) {
      percent = SmartDashboard.getNumber("Shooter Percent", 0.0);
    }
    shooter.setShooterPercentOutput(percent);
    log.writeLog(false, "ShooterSetPercent", "Percent", percent);
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
