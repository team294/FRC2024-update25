// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.Constants.TrajectoryConstants;
import frc.robot.subsystems.DriveTrain;
import frc.robot.utilities.AllianceSelection;
import frc.robot.utilities.BCRRobotState;
import frc.robot.utilities.FileLog;
import frc.robot.utilities.BCRRobotState.ShotMode;

import java.lang.Math;


public class DriveWithJoysticksAdvance extends Command {
  private final Joystick leftJoystick;
  private final Joystick rightJoystick;
  private final DriveTrain driveTrain;
  private final BCRRobotState robotState;
  private final FileLog log;
  private final AllianceSelection allianceSelection;
  private ProfiledPIDController turnRateController;
  private boolean firstInDeadband;
  private int logRotationKey;
  private double fwdVelocity, leftVelocity, turnRate, nextTurnRate;
  private double goalAngle;       // in radians
  private double startTime;
  private boolean firstCorrecting;
  private boolean aimLock = false;


    /**
   * @param leftJoystick left joystick.  X and Y axis control robot movement, relative to front of robot
   * @param rightJoystick right joystick.  X-axis controls robot rotation.
   * @param driveTrain drive train subsystem to use
   * @param log filelog to use
   */

  public DriveWithJoysticksAdvance(Joystick leftJoystick, Joystick rightJoystick, AllianceSelection allianceSelection, 
      DriveTrain driveTrain, BCRRobotState robotstate, FileLog log) {
    this.leftJoystick = leftJoystick;
    this.rightJoystick = rightJoystick;
    this.driveTrain = driveTrain;
    this.allianceSelection = allianceSelection;
    this.robotState = robotstate;
    this.log = log;
    turnRateController = new ProfiledPIDController(DriveConstants.kPJoystickThetaController, 0, 0, TrajectoryConstants.kThetaControllerConstraints);
    turnRateController.enableContinuousInput(-Math.PI, Math.PI);


    logRotationKey = log.allocateLogRotation();

    addRequirements(driveTrain);
  }

  @Override
  public void initialize() {
    driveTrain.setDriveModeCoast(false);

    goalAngle = driveTrain.getPose().getRotation().getRadians();

    firstInDeadband = true;
    firstCorrecting = true;

    turnRateController.reset(goalAngle);      // sets the current setpoint for the controller
    turnRateController.setGoal(goalAngle);    // set the goal for the controller
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    aimLock = driveTrain.isAimLockEnabled();

    fwdVelocity = -leftJoystick.getY();
    leftVelocity = -leftJoystick.getX();
    turnRate = -rightJoystick.getX();

    SmartDashboard.putNumber("Left Joystick Y", fwdVelocity);
    SmartDashboard.putNumber("Left Joystick X", leftVelocity);
    SmartDashboard.putNumber("Right Joystick X", turnRate);

    // Apply deadbands

    fwdVelocity = (Math.abs(fwdVelocity) < OIConstants.joystickDeadband) ? 0 : scaleJoystick(fwdVelocity) * SwerveConstants.kMaxSpeedMetersPerSecond;
    leftVelocity = (Math.abs(leftVelocity) < OIConstants.joystickDeadband) ? 0 : scaleJoystick(leftVelocity) * SwerveConstants.kMaxSpeedMetersPerSecond;
    turnRate = (Math.abs(turnRate) < OIConstants.joystickDeadband) ? 0 : scaleTurn(turnRate) * SwerveConstants.kMaxTurningRadiansPerSecond;

    // SmartDashboard.putNumber("Goal Angle", goalAngle);
    // SmartDashboard.putNumber("Current Angle", driveTrain.getPose().getRotation().getRadians());

    // Uses profiled PID controller if the joystick is in the deadband
    if(turnRate == 0 || aimLock){
      if(firstInDeadband){
        // goalAngle = driveTrain.getPose().getRotation().getRadians();
        // goalAngle = MathUtil.angleModulus(goalAngle);
        // turnRateController.reset(goalAngle);      // sets the current setpoint for the controller
        firstInDeadband = false;
        driveTrain.enableFastLogging(true);
        startTime = System.currentTimeMillis();
      }
      if(System.currentTimeMillis() - startTime > 100){
        if(firstCorrecting){
          firstCorrecting = false;
          driveTrain.enableFastLogging(false);
          goalAngle = driveTrain.getPose().getRotation().getRadians();
          goalAngle = MathUtil.angleModulus(goalAngle);
          turnRateController.reset(goalAngle);      // sets the current setpoint for the controller
        }
        if (aimLock) {
          if (robotState.getShotMode() == ShotMode.FAR_PASS  || robotState.getShotMode() == ShotMode.VISION_FAR_PASS) {
            // Aim towards far pass target
            goalAngle = Math.atan((driveTrain.getPose().getY() - allianceSelection.getFarPassYPos())/(driveTrain.getPose().getX() - allianceSelection.getFarPassXPos()));
          } else if (robotState.getShotMode() == ShotMode.VISION_MID_PASS) {
            // Aim towards mid pass target
            goalAngle = Math.atan((driveTrain.getPose().getY() - allianceSelection.getMidPassYPos())/(driveTrain.getPose().getX() - allianceSelection.getMidPassXPos()));
          } else {
            // Aim towards speaker
            goalAngle = Math.atan((driveTrain.getPose().getY() - allianceSelection.getSpeakerYPos())/driveTrain.getPose().getX());
          }
          goalAngle = MathUtil.angleModulus(goalAngle);
          SmartDashboard.putNumber("Goal Angle", goalAngle);
          turnRateController.reset(goalAngle);
        }
      // When the right button on the right joystick is pressed then the robot turns pi radians(180 degrees)
      // This button works but it is currently used for other commands
      // if(rightJoystick.getRawButtonPressed(2)){
      //   goalAngle += Math.PI;
      //   MathUtil.angleModulus(goalAngle);
      // }

      // When the left button on the right joystick is pressed then the robot goes to 0 radians absolute
      // This button works but it is currently used for other commands
      // goalAngle = rightJoystick.getRawButtonPressed(1) ? 0 : goalAngle;

      // Calculates using the profiledPIDController what the next speed should be
      SmartDashboard.putNumber("Goal Position", goalAngle);
      SmartDashboard.putNumber("Get Position", driveTrain.getPose().getRotation().getRadians());
      SmartDashboard.putNumber("Angle Error", goalAngle - driveTrain.getPose().getRotation().getRadians());
      SmartDashboard.putBoolean("In Angle Deadband", Math.abs(goalAngle - driveTrain.getPose().getRotation().getRadians()) < Math.PI/180);
      if(Math.abs(goalAngle - driveTrain.getPose().getRotation().getRadians()) > Math.PI/180){
        nextTurnRate = turnRateController.calculate(driveTrain.getPose().getRotation().getRadians(), goalAngle);
      }
      else{
        nextTurnRate = 0;
      }
      if(log.isMyLogRotation(logRotationKey)) {
        log.writeLog(false, "DriveWithJoystickAdvance", "Joystick", "Fwd", fwdVelocity, "Left", leftVelocity, "Turn", nextTurnRate, "Goal Angle", goalAngle);
      }
    
      if(aimLock){ 
        nextTurnRate *= 2.0;
      }
      driveTrain.drive(fwdVelocity, leftVelocity, nextTurnRate, true, false);

      //firstInDeadband = false;
      }else{
        driveTrain.drive(fwdVelocity, leftVelocity, turnRate, true, false);
        
      }
    }

    // Just uses the regular turnRate if the joystick is not in the deadband
    else{
      if(log.isMyLogRotation(logRotationKey)) {
        log.writeLog(false, "DriveWithJoystickAdvance", "Joystick", "Fwd", fwdVelocity, "Left", leftVelocity, "Turn", turnRate);
      }

      
      driveTrain.drive(fwdVelocity, leftVelocity, turnRate, true, false);
      
      firstInDeadband = true;
      firstCorrecting = true;
    }
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  /**
   * Re-maps joystick value to better enable fine robot control at small joystick
   * values (low speeds) and full-speed travel at large joystick values.
   * This method is optimized for linear travel.
   * @param rawJoystick Raw joystick value, -1.0 to +1.0
   * @return Scaled joystick value, -1.0 to +1.0
   */
  private double scaleTurn(double rawJoystick){
    return Math.signum(rawJoystick)*(0.6801 * rawJoystick * rawJoystick + 0.3232 * Math.abs(rawJoystick) - 0.0033);
  }

  /**
   * Re-maps joystick value to better enable fine robot control at small joystick
   * values (low speeds) and full-speed travel at large joystick values.
   * This method is optimized for rotating the robot.
   * @param rawJoystick Raw joystick value, -1.0 to +1.0
   * @return Scaled joystick value, -1.0 to +1.0
   */
  private double scaleJoystick(double rawJoystick){
    return Math.signum(rawJoystick)*(0.7912*rawJoystick*rawJoystick + 0.2109*Math.abs(rawJoystick) - 0.0022);
  }
}


