// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.WristConstants;
import frc.robot.Constants.WristConstants.WristAngle;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Wrist;
import frc.robot.utilities.FileLog;

public class WristXboxControl extends Command {
  private final CommandXboxController xboxController;
  private final Wrist wrist;
  private final Intake intake;
  private final Feeder feeder;
  private final FileLog log;

  /**
   * Controls the elevator and wrist using the left and right XBox Controller joysticks
   * @param xboxController
   * @param elevator
   * @param wrist
   * @param log
   */
  public WristXboxControl(CommandXboxController xboxController, Wrist wrist, Intake intake, Feeder feeder, FileLog log) {
    this.wrist = wrist;
    this.intake = intake;
    this.feeder = feeder;
    this.xboxController = xboxController;
    this.log = log;

    addRequirements(wrist, intake, feeder);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    log.writeLog(false, "WristXboxControl", "Initialize");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double wristPct = -xboxController.getRightY();
    if (Math.abs(wristPct)<OIConstants.manualWristDeadband) wristPct=0;
    wristPct *= WristConstants.maxPercentOutput;

    // If wrist is above intake limit, stop the intaking
    if (wrist.getWristAngle() > WristAngle.intakeLimit.value) {
      intake.setCenteringMotorPercentOutput(0);
      intake.setIntakePercentOutput(0);
      feeder.setFeederPercentOutput(0);
    }

    log.writeLog(false, "WristXboxControl", "Execute", "Right Xbox", wristPct);

    wrist.setWristMotorPercentOutput(wristPct);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    wrist.stopWrist();

    log.writeLog(false, "WristXboxControl", "End");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
