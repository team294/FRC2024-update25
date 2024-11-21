// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;
import frc.robot.utilities.FileLog;

public class IntakeSetPercent extends Command {
  private final FileLog log;
  private final Intake intake;
  private double intakePercent = 0.0;
  private double centeringPercent;
  private boolean fromShuffleboard;

  /**
   * Sets the percent output to the Intake and Centering motors from Shuffleboard
   * and ends immediately.
   * @param intake intake subsystem
   * @param log
   */
  public IntakeSetPercent(Intake intake, FileLog log) {
    this.log = log;
    this.intake = intake;
    addRequirements(intake);
    this.fromShuffleboard = true;

    if(SmartDashboard.getNumber("Intake Percent", -9999.9) == -9999.9) {
      SmartDashboard.putNumber("Intake Percent", 0);
    }
    if(SmartDashboard.getNumber("Centering Percent", -9999.9) == -9999.9) {
      SmartDashboard.putNumber("Centering Percent", 0);
    }
  }

  /**
   * Sets the percent output to the Intake and Centering motors
   * and ends immediately.
   * @param intakePercent -1.0 to 1.0 (+ = intake, - = outtake)
   * @param centeringPercent -1.0 to 1.0 (+ = intake, - = outtake)
   * @param intake intake subsystem
   * @param log
   */  
  public IntakeSetPercent(double intakePercent, double centeringPercent, Intake intake, FileLog log) {
    this.log = log;
    this.intake = intake;
    addRequirements(intake);
    this.fromShuffleboard = false;
    this.intakePercent = intakePercent;
    this.centeringPercent = centeringPercent;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (fromShuffleboard) {
      intakePercent = SmartDashboard.getNumber("Intake Percent", 0.0);
      centeringPercent = SmartDashboard.getNumber("Centering Percent", 0.0);
    }
    intake.setIntakePercentOutput(intakePercent);
    intake.setCenteringMotorPercentOutput(centeringPercent);

    log.writeLog(false, "IntakeSetPercent", "Initialize", "Intake Percent", intakePercent,
      "Centering Percent", centeringPercent);
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
