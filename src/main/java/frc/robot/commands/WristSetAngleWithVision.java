/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.RobotDimensions;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Wrist;
import frc.robot.utilities.AllianceSelection;
import frc.robot.utilities.FileLog;

public class WristSetAngleWithVision extends Command {

  private double angle;
  private final AllianceSelection allianceSelection;
  private final DriveTrain driveTrain;
  private final Wrist wrist;
  private final FileLog log;

  /**
   * Moves wrist to target angle.  Command ends when wrist is within 5 degrees of the target position.
   * <p> This command does nothing and immediately returns if the wrist is not calibrated.
   * @param angle target angle in degrees.  (0 = horizontal in front of robot, + = up, - = down)
   */
  public WristSetAngleWithVision(Wrist wrist, AllianceSelection allianceSelection, DriveTrain drivetrain, FileLog log) {
    this.wrist = wrist;
    this.allianceSelection = allianceSelection;
    this.driveTrain = drivetrain;
    this.log = log;

    SmartDashboard.putNumber("Wrist Vision Constant Offset", 0);

    // Print out table of arm angles versus distance
    double x;
    double y = allianceSelection.getSpeakerYPos();
    for (x = 0; x <= 6; x += 0.1) {
      log.writeLog(true, "WristSetAngleWithVision", "Initialize", "x", x, "y", y,
        "angle", getAngleFromDistance(3, x, y));
    }

    addRequirements(wrist);
  }

  /**
   * Calculates the desired arm angle for the speaker shot using a polynomial expression, based on robot location passed in
   * @param xRobot robot X location on field, in meters
   * @param yRobot robot X location on field, in meters
   * @return Recommended wrist angle, in degrees 
   */
  private double getAngleFromDistanceSimplified(double xRobot, double yRobot) {
    // distance from speaker
    double x = xRobot;
    double y = yRobot - allianceSelection.getSpeakerYPos();
    double dist = Math.sqrt(x*x+y*y);

    // angle using distance and calibrated polynomial expression
    double angle = ((-0.2118*dist + 3.8400)*dist - 24.132)*dist - 19.87; //F3 Reduced Angle by 2 degree from decrease by 17.87 to 19.87 
    return angle;
  }

  /**
   * Calculates the desired arm angle for the speaker shot, based on robot location on the field
   * @param n the number of recursions the function runs before using base condition
   * @return Recommended wrist angle, in degrees 
   */
  private double getAngleFromDistance(int n) {
    return getAngleFromDistance(n, driveTrain.getPose().getX(), driveTrain.getPose().getY());
  }

  /**
   * Calculates the desired arm angle for the speaker shot, based on robot location passed in
   * @param n the number of recursions the function runs before using base condition
   * @param xRobot robot X location on field, in meters
   * @param yRobot robot X location on field, in meters
   * @return Recommended wrist angle, in degrees 
   */
  private double getAngleFromDistance(int n, double xRobot, double yRobot) {
    if (n == 0) return 0;

    // distance from speaker
    double x = xRobot;
    double y = yRobot - allianceSelection.getSpeakerYPos();
    double dist = Math.sqrt(x*x+y*y);

    // Fix for short angles
    if (dist<=1.0) {
      return (-4.5518)*dist -36.2;
    }

    double armAngleIterateRadians = Units.degreesToRadians(getAngleFromDistance(n-1, xRobot, yRobot));

    // distance from center of robot to shooter
    double distOff = RobotDimensions.lengthOfArmFromWristPivotToCenterPathOfShooter*Math.cos(armAngleIterateRadians);

    double heightOfShooter = RobotDimensions.heightFromGroundToWristPivot+RobotDimensions.lengthOfArmFromWristPivotToCenterPathOfShooter*Math.sin(armAngleIterateRadians);

    // correction offset calculated from regression
    double correctionOffset = (n == 3) ? (3.79204*dist - 12.7572 + 1) : 0;

    return Units.radiansToDegrees(Math.atan((FieldConstants.heightOfSpeaker-heightOfShooter)/(dist - distOff))) - 90 - 10 + correctionOffset;
  } 

  // Called just before this Command runs the first time
  @Override
  public void initialize() {
    log.writeLog(false, "WristSetAngleWithVision", "Initialize", "Target", angle);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  public void execute() {
    try {
      angle = getAngleFromDistanceSimplified(driveTrain.getPose().getX(), driveTrain.getPose().getY());
      wrist.setWristAngle(angle + SmartDashboard.getNumber("Wrist Vision Constant Offset", 0));
      wrist.updateWristLog(false);
    } catch (ArithmeticException e) {
      return;
    }
  }

  // Called once after isFinished returns true
  @Override
  public void end(boolean interrupted) {
    log.writeLog(false, "WristSetAngleWithVision", "End", "Target", angle, "Current angle", wrist.getWristAngle());
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  public boolean isFinished() {
    return !wrist.isEncoderCalibrated();
  }
}