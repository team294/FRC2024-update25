// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.subsystems.DriveTrain;
import frc.robot.utilities.AllianceSelection;
import frc.robot.utilities.FileLog;

public class DriveWithController extends Command {
  private DriveTrain driveTrain;
  private CommandXboxController xboxController;
  private final AllianceSelection allianceSelection;
  private FileLog log;
  private double fwdVelocity, leftVelocity, turnRate;

  /**
   * Controls the driveTrain using the left and right joysticks on an Xbox controller.
   * @param driveTrain DriveTrain subsystem
   * @param xboxController Xbox controller. Left joystick X and Y axis control robot movement, relative to the field from the 
   *   perspective of the current Alliance's driver station. Right joystick X-axis controls robot rotation.
   * @param allianceSelection AllianceSelection utility
   * @param log FileLog utility
   */
  public DriveWithController(DriveTrain driveTrain, CommandXboxController xboxController, AllianceSelection allianceSelection, FileLog log) {
    this.driveTrain = driveTrain;
    this.xboxController = xboxController;
    this.allianceSelection = allianceSelection;
    this.log = log;
    addRequirements(driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    log.writeLog(false, "DriveWithController", "Init");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    fwdVelocity = allianceSelection.getAlliance() == Alliance.Blue ? -xboxController.getLeftY() : xboxController.getLeftY();
    leftVelocity = allianceSelection.getAlliance() == Alliance.Blue ? -xboxController.getLeftX() : xboxController.getLeftX();
    turnRate = -xboxController.getRightX();

    fwdVelocity = (Math.abs(fwdVelocity) < OIConstants.joystickDeadband) ? 0 : scaleJoystick(fwdVelocity) * SwerveConstants.kMaxSpeedMetersPerSecond;
    leftVelocity = (Math.abs(leftVelocity) < OIConstants.joystickDeadband) ? 0 : scaleJoystick(leftVelocity) * SwerveConstants.kMaxSpeedMetersPerSecond;
    turnRate = (Math.abs(turnRate) < OIConstants.joystickDeadband) ? 0 : scaleTurn(turnRate) * SwerveConstants.kMaxTurningRadiansPerSecond;
    
    driveTrain.drive(fwdVelocity, leftVelocity, turnRate, true, false);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end
  @Override
  public boolean isFinished() {
    return false;
  }

  /**
   * Re-maps joystick value to better enable fine robot control at small joystick values (low speeds) and full-speed travel at large joystick values.
   * This method is optimized for linear travel.
   * @param rawJoystick raw joystick value, -1.0 to +1.0
   * @return scaled joystick value, -1.0 to +1.0
   */
  private double scaleTurn(double rawJoystick) {
    return Math.signum(rawJoystick) * (0.6801 * rawJoystick * rawJoystick + 0.3232 * Math.abs(rawJoystick) - 0.0033);
  }

  /**
   * Re-maps joystick value to better enable fine robot control at small joystick values (low speeds) and full-speed travel at large joystick values.
   * This method is optimized for rotating the robot.
   * @param rawJoystick raw joystick value, -1.0 to +1.0
   * @return scaled joystick value, -1.0 to +1.0
   */
  private double scaleJoystick(double rawJoystick) {
    return Math.signum(rawJoystick) * (0.7912 * rawJoystick * rawJoystick + 0.2109 * Math.abs(rawJoystick) - 0.0022);
  }
}