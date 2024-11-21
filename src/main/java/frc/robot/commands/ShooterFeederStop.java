// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Shooter;
import frc.robot.utilities.FileLog;

public class ShooterFeederStop extends Command {

  private final Shooter shooter;
  private final Feeder feeder;
  private final FileLog log;

  /** Creates a new IntakeStop. */
  public ShooterFeederStop(Shooter shooter, Feeder feeder, FileLog log) {
    this.shooter = shooter;
    this.feeder = feeder;
    this.log = log;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shooter, feeder);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    shooter.stopMotors();
    feeder.stopFeeder();
    log.writeLog(false, "ShooterFeederStop", "Init");
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
