// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.StatusSignal;
// import com.ctre.phoenix6.configs.Pigeon2Configuration;
// import com.ctre.phoenix6.configs.Pigeon2Configurator;
import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.Ports.*;

import static frc.robot.Constants.DriveConstants.*;

import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.Ports;
import frc.robot.Constants.SwerveConstants;
import frc.robot.utilities.*;

// Vision imports
import org.photonvision.EstimatedRobotPose;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import java.util.Optional;


public class DriveTrain extends SubsystemBase implements Loggable {

  // File logging variables
  private FileLog log;
  private int logRotationKey;         // key for the logging cycle for this subsystem
  private boolean fastLogging = false; // true is enabled to run every cycle; false follows normal logging cycles

  // variables for swerve modules
  private final SwerveModule swerveFrontLeft;
  private final SwerveModule swerveFrontRight;
  private final SwerveModule swerveBackLeft;
  private final SwerveModule swerveBackRight;
    
  // variables for gyro and gyro calibration
  private final Pigeon2 pigeon = new Pigeon2(CANPigeonGyro, Ports.CANivoreBus);
  // private final Pigeon2Configurator pigeonConfigurator = pigeon.getConfigurator();
  // private Pigeon2Configuration pigeonConfig;
  private final StatusSignal<Angle> pigeonYaw = pigeon.getYaw();
  private final StatusSignal<Angle> pigeonPitch = pigeon.getRoll();    // Pigeon is mounted rotated by 90deg, so robot pitch is pigeon roll
  private final StatusSignal<Boolean> pigeonFault = pigeon.getFault_Hardware();
  private double yawZero = 0.0;
  private double pitchZero = 0.0;
  private final Matrix<N3, N1> closeMatrix = new Matrix<>(Nat.N3(), Nat.N1(), new double[] {.9,.9,.9});
  private final Matrix<N3, N1> farMatrix = new Matrix<>(Nat.N3(), Nat.N1(), new double[] {2,2,2});


  // variables to help calculate angular velocity for turnGyro
  // private double prevAng; // last recorded gyro angle
  // private double currAng; // current recorded gyro angle
  // private double prevTime; // last time gyro angle was recorded
  // private double currTime; // current time gyro angle is being recorded
  // private double angularVelocity;  // Robot angular velocity in degrees per second
  // private LinearFilter lfRunningAvg = LinearFilter.movingAverage(4); //calculate running average to smooth quantization error in angular velocity calc

  // variable to store vision camera
  private PhotonCameraWrapper camera;
  private NotePhotonCameraWrapper noteCamera;

  // false = will not use vision in odometry, true = uses vision for odometry
  private boolean useVisionForOdometry = false;

  // variable for vison-based aiming in DriveWithJoysticksAdvance
  private boolean aimLock = false;

  // Odometry class for tracking robot pose
  private final SwerveDrivePoseEstimator poseEstimator; 
  private final Field2d field = new Field2d();    // Field to dispaly on Shuffleboard
  private double speedAvg;        // Average speed of the robot chassis

  //Slew rate limiter
  // private boolean elevatorUpPriorIteration = false;       // Tracking for elevator position from prior iteration


  /**
   * Constructs the DriveTrain subsystem
   * @param log FileLog object for logging
   */
  public DriveTrain(AllianceSelection allianceSelection, FileLog log) {
    this.log = log; // save reference to the fileLog
    logRotationKey = log.allocateLogRotation();     // Get log rotation for this subsystem
    this.camera = new PhotonCameraWrapper(allianceSelection, log, logRotationKey);
    this.noteCamera = new NotePhotonCameraWrapper(log, logRotationKey);
    
    // enable cameras
    cameraInit();

    // create swerve modules
    swerveFrontLeft = new SwerveModule("FL",
      CANDriveFrontLeftMotor, CANDriveTurnFrontLeftMotor, CANTurnEncoderFrontLeft, false, true,
      false, offsetAngleFrontLeftMotor, SwerveConstants.kVmFL, log);
    swerveFrontRight = new SwerveModule("FR",
      CANDriveFrontRightMotor, CANDriveTurnFrontRightMotor, CANTurnEncoderFrontRight, false, true,
      false, offsetAngleFrontRightMotor, SwerveConstants.kVmFR, log);
    swerveBackLeft = new SwerveModule("BL",
      CANDriveBackLeftMotor, CANDriveTurnBackLeftMotor, CANTurnEncoderBackLeft, false, true,
      false, offsetAngleBackLeftMotor, SwerveConstants.kVmBL, log);
    swerveBackRight = new SwerveModule("BR",
      CANDriveBackRightMotor, CANDriveTurnBackRightMotor, CANTurnEncoderBackRight, false, true,
      false, offsetAngleBackRightMotor, SwerveConstants.kVmBR, log);

    // Put drive mode on shuffleboard
    setDriveModeCoast(false);

    // configure gyro
    // This Pigeon is mounted normally, so no need to change orientation
    // pigeonConfig.MountPose.MountPoseYaw = 0;
    // pigeonConfig.MountPose.MountPosePitch = 0;
    // pigeonConfig.MountPose.MountPoseRoll = 0;
    // This Pigeon has no need to trim the gyro
    // pigeonConfig.GyroTrim.GyroScalarX = 0;
    // pigeonConfig.GyroTrim.GyroScalarY = 0;
    // pigeonConfig.GyroTrim.GyroScalarZ = 0;
    // We want the thermal comp and no-motion cal enabled, with the compass disabled for best behavior
    // pigeonConfig.Pigeon2Features.DisableNoMotionCalibration = false;
    // pigeonConfig.Pigeon2Features.DisableTemperatureCompensation = false;
    // pigeonConfig.Pigeon2Features.EnableCompass = false;
    // pigeonConfigurator.apply(pigeonConfig);

    // zero gyro and initialize angular velocity variables
    zeroGyro();
    // prevAng = getGyroRaw();
    // currAng = getGyroRaw();
    // prevTime = System.currentTimeMillis();
    // currTime = System.currentTimeMillis();
    // lfRunningAvg.reset();

    // create and initialize odometery
    // Set initial location to 0,0.
    poseEstimator = new SwerveDrivePoseEstimator(kDriveKinematics, Rotation2d.fromDegrees(getGyroRotation()), 
       getModulePositions(), new Pose2d(0, 0, Rotation2d.fromDegrees(0)) );
    // poseEstimator.setVisionMeasurementStdDevs(); // TODO
    SmartDashboard.putData("Field", field);
  }
  

  // ************ Gryo methods

  /**
   * Verifies if Gyro is still reading
   * @return true = gryo is connected to Rio
   */
  public boolean isGyroReading() {
    return !pigeonFault.refresh().getValue();
  }

  /**
   * Gets the raw gyro angle (can be greater than 360).
   * Angle from gyro is so that + = left and - = right
   * @return raw gyro angle, in degrees.
   */
  public double getGyroRaw() {
    return pigeonYaw.refresh().getValueAsDouble();
  }

  /**
	 * @return double, gyro pitch from 180 to -180, in degrees (postitive is nose up, negative is nose down)
	 */
	public double getGyroPitchRaw() {
		return pigeonPitch.refresh().getValueAsDouble();
  }

  public void resetGyroPitch(){
    pitchZero = getGyroPitchRaw();
  }

  /**
	 * Zero the gyro position in software to the current angle.
	 */
	public void zeroGyro() {
    yawZero = getGyroRaw(); // set yawZero to gyro angle
    pitchZero = getGyroPitchRaw();  // set pitchZero to gyro pitch
  }
  
  /**
	 * Zero the gyro position in software against a specified angle.
	 * @param currentHeading current robot angle compared to the zero angle
	 */
	public void zeroGyroRotation(double currentHeading) {
		// set yawZero to gryo angle, offset to currentHeading
		yawZero = getGyroRaw() - currentHeading;
  }

  /**
	 * @return double, gyro angle from 180 to -180, in degrees (postitive is left, negative is right)
	 */
	public double getGyroRotation() {
		double angle = getGyroRaw() - yawZero;
		// Angle will be in terms of raw gyro units (-inf,inf), so you need to convert to (-180, 180]
		angle = MathBCR.normalizeAngle(angle);
		return angle;
  }

  /**
	 * @return double, gyro pitch from 180 to -180, in degrees (postitive is nose up, negative is nose down)
	 */
	public double getGyroPitch() {
		return getGyroPitchRaw() - pitchZero;
  }

  /**
   * @return gyro angular velocity (with some averaging to reduce noise), in degrees per second.
   * Positive is turning left, negative is turning right.
   */
  public double getAngularVelocity () {
    // return angularVelocity;
    //return -pigeon.getRate();     // TODO check if this is accurate!  If so, then delete the commented-out code to calc angularVelocity in periodic, constructor, etc
    return pigeon.getAngularVelocityZWorld().refresh().getValueAsDouble(); //getRate was deprecated, negative removed as getRate returns cw while this returns ccw
  }

  /**
   * @return angular velocity from motor velocity readings (NOT from gyro)
   * Positive is turning left, negative is turning right.
   */
  // public double getAngularVelocityFromWheels () {
    // In the 2022 code, this was more accurate than the angular velocity from
    // the gyro.  This was used in the DriveTurnGyro code.  However, angular velocity
    // was easy to calculate from a west coast driveTrain.  How do we calculate this
    // from a swerve drive train?  Do we need this method?
  //   return ((getRightEncoderVelocity() - getLeftEncoderVelocity()) / 2) * wheelInchesToGyroDegrees;
  // }


  // ************ Swerve drive methods

  /**
   * Configures the motors and encoders for every swerve module.
   * The swerve modules are automatically configured in the SwerveModule constructors.  So, this
   * method should not need to be called.
   * <p>However, if the robot browns-out or otherwise partially resets, then this can be used to 
   * force the motors and encoders to have the right calibration and settings, especially the
   * calibration angle for each swerve module.
   * <p> <b>Note</b> that this procedure includes multiple blocking calls and will delay robot code.
   */
  public void configureSwerveModules(){
    swerveFrontLeft.configSwerveModule();
    swerveFrontRight.configSwerveModule();
    swerveBackLeft.configSwerveModule();
    swerveBackRight.configSwerveModule();
  }

  /**
   * Sets the drive train to coast or brake mode.
   * <p> <b>Note</b> that this procedure includes multiple blocking calls and will delay robot code by ~250ms if the mode is changed.
   * However, if setCoast is the same as the current setting, then nothing is sent to the swerve modules and there will not
   * be a delay.
   * @param setCoast true = coast mode, false = brake mode
   */
  public void setDriveModeCoast(boolean setCoast) {
    swerveFrontLeft.setMotorModeCoast(setCoast);
    swerveFrontRight.setMotorModeCoast(setCoast);
    swerveBackLeft.setMotorModeCoast(setCoast);
    swerveBackRight.setMotorModeCoast(setCoast);

    SmartDashboard.putString("Drive Mode", setCoast ? "Coast" : "Brake");
  }
  /**
   * Gets the drive train mode (coast vs brake).
   * @return Returns true if in motors are in coast, or false if in brake.
   */
  public boolean isDriveModeCoast(){
    return swerveFrontLeft.isMotorModeCoast() && swerveFrontRight.isMotorModeCoast() &&
      swerveBackLeft.isMotorModeCoast() && swerveBackLeft.isMotorModeCoast();
  }

  /**
   * @param percentOutput Percent output to motor, -1 to +1
   */
  public void setDriveMotorsOutput(double percentOutput){
    swerveFrontLeft.setDriveMotorPercentOutput(percentOutput);
    swerveFrontRight.setDriveMotorPercentOutput(percentOutput);
    swerveBackLeft.setDriveMotorPercentOutput(percentOutput);
    swerveBackRight.setDriveMotorPercentOutput(percentOutput);
  }
  

  /**
   * 
   * @param percentOutput Percent output to motor, -1 to +1
   */
  public void setTurningMotorsOutput(double percentOutput){
    swerveFrontLeft.setTurnMotorPercentOutput(percentOutput);
    swerveFrontRight.setTurnMotorPercentOutput(percentOutput);
    swerveBackLeft.setTurnMotorPercentOutput(percentOutput);
    swerveBackRight.setTurnMotorPercentOutput(percentOutput);
  }


  /**
   * Sets the facing of the wheels (direction the wheels are pointing).  Note that this sets the absolute facing.  It
   * does *not* turn the wheels to the "optimized" facing +/-180 degrees if that is closer.
   * @param angle Desired wheel facing relative to front of chassis in degrees, -180 to +180 (+=left, -=right, 0=facing front of robot)
   */
  public void setWheelFacings(double angle){
    swerveFrontLeft.setWheelFacing(angle);
    swerveFrontRight.setWheelFacing(angle);
    swerveBackLeft.setWheelFacing(angle);
    swerveBackRight.setWheelFacing(angle);
  }


  /**
   * Stops all of the drive and turning motors
   */
  public void stopMotors() {
    swerveFrontLeft.stopMotors();
    swerveFrontRight.stopMotors();
    swerveBackLeft.stopMotors();
    swerveBackRight.stopMotors();
  }

  /**
   * Sets the swerve ModuleStates.
   *
   * @param desiredStates The desired SwerveModule states.          
   * 0 = FrontLeft, 1 = FrontRight, 2 = BackLeft, 3 = BackRight
   * @param isOpenLoop true = fixed drive percent output to approximate velocity, false = closed loop drive velocity control
   */
  public void setModuleStates(SwerveModuleState[] desiredStates, boolean isOpenLoop) {

    // Desaturate wheel speeds to a little below max speed.  It takes a while to accelerate to
    // max speed, so reducing the max will help movement accuracy.
    SwerveDriveKinematics.desaturateWheelSpeeds(
        desiredStates, SwerveConstants.kMaxSpeedMetersPerSecond);

    // Convert states to chassisspeeds
    ChassisSpeeds chassisSpeeds = kDriveKinematics.toChassisSpeeds(desiredStates);
    double xSlewed, omegaLimited, ySlewed;
    omegaLimited = chassisSpeeds.omegaRadiansPerSecond;
    xSlewed = chassisSpeeds.vxMetersPerSecond;
    ySlewed = chassisSpeeds.vyMetersPerSecond;

    // convert back to swervem module states
    desiredStates = kDriveKinematics.toSwerveModuleStates(new ChassisSpeeds(xSlewed, ySlewed, omegaLimited), new Translation2d());
    
    swerveFrontLeft.setDesiredState(desiredStates[0], isOpenLoop);
    swerveFrontRight.setDesiredState(desiredStates[1], isOpenLoop);
    swerveBackLeft.setDesiredState(desiredStates[2], isOpenLoop);
    swerveBackRight.setDesiredState(desiredStates[3], isOpenLoop);
  }

  // TODO Add version of setModuleStates with acceleration

  /**
   * Reads the current swerve ModuleStates.
   * @return The current module states, as measured by the encoders.  
   * 0 = FrontLeft, 1 = FrontRight, 2 = BackLeft, 3 = BackRight
   */
  public SwerveModuleState[] getModuleStates() {
    return new SwerveModuleState[] {
      swerveFrontLeft.getState(), swerveFrontRight.getState(),
      swerveBackLeft.getState(), swerveBackRight.getState()
    };
  }

  /**
   * Returns the speed of the chassis in X, Y, and theta <b>in the robot frame of reference</b>.
   * <p> Speed of the robot in the x direction, in meters per second (+ = forward)
   * <p> Speed of the robot in the y direction, in meters per second (+ = move to the left)
   * <p> Angular rate of the robot, in radians per second (+ = turn to the left)
   * @return ChassisSpeeds object representing the chassis speeds.
   */
  public ChassisSpeeds getChassisSpeeds() {
    return kDriveKinematics.toChassisSpeeds(getModuleStates());
  }

  /**
   * Reads the current swerve ModulePositions.
   * @return The current module positions, as measured by the encoders.  
   * 0 = FrontLeft, 1 = FrontRight, 2 = BackLeft, 3 = BackRight
   */
  public SwerveModulePosition[] getModulePositions() {
    return new SwerveModulePosition[] {
      swerveFrontLeft.getPosition(), swerveFrontRight.getPosition(),
      swerveBackLeft.getPosition(), swerveBackRight.getPosition()
    };
  }
  
  /**
   * Method to drive the robot using desired robot velocity and orientation, such as from joystick info.
   *
   * @param xSpeed Speed of the robot in the x direction, in meters per second (+ = forward)
   * @param ySpeed Speed of the robot in the y direction, in meters per second (+ = move to the left)
   * @param rot Angular rate of the robot, in radians per second (+ = turn to the left)
   * @param fieldRelative True = the provided x and y speeds are relative to the field. False = the provided x and y speeds are relative to the current facing of the robot.
   * @param isOpenLoop true = fixed drive percent output to approximate velocity, false = closed loop drive velocity control 
   */
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative, boolean isOpenLoop) {
    drive(xSpeed, ySpeed, rot, new Translation2d(), fieldRelative, isOpenLoop);
  }

  /**
   * Method to drive the robot using desired robot velocity and orientation, such as from joystick info.
   *
   * @param xSpeed Speed of the robot in the x direction, in meters per second (+ = forward)
   * @param ySpeed Speed of the robot in the y direction, in meters per second (+ = move to the left)
   * @param rot Angular rate of the robot, in radians per second (+ = turn to the left)
   * @param centerOfRotationMeters The center of rotation. For example, if you set the center of
   *     rotation at one corner of the robot and XSpeed, YSpeed = 0, then the robot will rotate around that corner.
   *     This feature may not work if fieldRelative = True (need to test).
   * @param fieldRelative True = the provided x and y speeds are relative to the field.
   * @param isOpenLoop true = fixed drive percent output to approximate velocity, false = closed loop drive velocity control
   * False = the provided x and y speeds are relative to the current facing of the robot. 
   */
   public void drive(double xSpeed, double ySpeed, double rot, Translation2d centerOfRotationMeters, boolean fieldRelative, boolean isOpenLoop) {
    
    ChassisSpeeds chassisSpeed;
    chassisSpeed = fieldRelative 
      ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, Rotation2d.fromDegrees(getGyroRotation()))
      : new ChassisSpeeds(xSpeed, ySpeed, rot);

    SwerveModuleState[] swerveModuleStates =
        kDriveKinematics.toSwerveModuleStates(
            chassisSpeed,
            centerOfRotationMeters);

    setModuleStates(swerveModuleStates, isOpenLoop);
  }

  // ************ Odometry methods

  /**
   * Returns the currently-estimated position of the robot on the field
   *
   * @return The robot's pose
   */
  public Pose2d getPose() {
    return poseEstimator.getEstimatedPosition();
  }

  /**
   * Resets the pose estimator to the specified pose.  I.e. defines the robot's
   * position and orientation on the field.
   * This method also resets the gyro, which is required for the pose
   * to properly reset.
   *
   * @param pose The pose to which to set the pose estimator.  Pose components include
   *    <p> Robot X location in the field, in meters (0 = field edge in front of driver station, + = away from our drivestation)
   *    <p> Robot Y location in the field, in meters (0 = right edge of field when standing in driver station, + = left when looking from our drivestation)
   *    <p> Robot angle on the field (0 = facing away from our drivestation, + to the left, - to the right)
   */
  public void resetPose(Pose2d pose) {
    zeroGyroRotation(pose.getRotation().getDegrees());
    poseEstimator.resetPosition( Rotation2d.fromDegrees(getGyroRotation()),
    getModulePositions(), pose );
  }
  
  /**
   * Returns the speed of the robot in X, Y, and theta <b>in the field frame of reference</b>.
   * <p> Speed of the robot in the x direction, in meters per second (+ = away from our drivestation)
   * <p> Speed of the robot in the y direction, in meters per second (+ = left when looking from our drivestation)
   * <p> Angular rate of the robot, in radians per second (+ = turn to the left)
   * @return ChassisSpeeds object representing the chassis speeds.
   */
  public ChassisSpeeds getRobotSpeeds() {
    return ChassisSpeeds.fromRobotRelativeSpeeds(getChassisSpeeds(), Rotation2d.fromDegrees(getGyroRotation()));
  }


  // ************ Information methods

  /**
   * Turns file logging on every scheduler cycle (~20ms) or every 10 cycles (~0.2 sec)
   * @param enabled true = every cycle, false = every 10 cycles
   */ 
  @Override
  public void enableFastLogging(boolean enabled) {
    fastLogging = enabled;
    camera.enableFastLogging(enabled);
  }

  /**
   * Checks if the CAN bus and gyro are working.  Sometimes, when the robot boots up, either the CAN bus or
   * the gyro don't initialize properly.  ROBOT CODE WILL NOT BE ABLE TO CONTROL MOTORS when this happens, so
   * always check this before starting a match!
   * @return true = something is not working.  false = CAN bus and gyro are both working.
   */
  public boolean canBusError() {
    return ((swerveFrontLeft.getDriveBusVoltage() < 7.0) || (swerveFrontLeft.getDriveTemp() < 5.0) || !isGyroReading());
  }


  @Override
  public void periodic() {
    
    // This method will be called once per scheduler run
    
    // save current angle and time for calculating angVel
    // currAng = getGyroRaw();
    // currTime = System.currentTimeMillis();
 
    // calculate angVel in degrees per second
    // angularVelocity =  lfRunningAvg.calculate( (currAng - prevAng) / (currTime - prevTime) * 1000 );

    // update 
    updateOdometry();
    
    if(fastLogging || log.isMyLogRotation(logRotationKey)) {
      updateDriveLog(false);

      if(!isGyroReading()) {
        RobotPreferences.recordStickyFaults("Gyro", log);
      }

      ChassisSpeeds robotSpeeds = getRobotSpeeds();

      // SmartDashboard.putNumber("Drive Average Dist in Meters", Units.inchesToMeters(getAverageDistance()));
      SmartDashboard.putNumber("Drive Speed", speedAvg);
      SmartDashboard.putNumber("Drive X Velocity", robotSpeeds.vxMetersPerSecond);
      SmartDashboard.putNumber("Drive Y Velocity", robotSpeeds.vyMetersPerSecond);
      SmartDashboard.putBoolean("Drive isGyroReading", isGyroReading());
      SmartDashboard.putNumber("Drive Raw Gyro", getGyroRaw());
      SmartDashboard.putNumber("Drive Gyro Rotation", getGyroRotation());
      SmartDashboard.putNumber("Drive AngVel", getAngularVelocity());
      SmartDashboard.putNumber("Drive Pitch", getGyroPitch());
      
      // position from poseEstimator (helpful for autos)
      Pose2d pose = poseEstimator.getEstimatedPosition();
      SmartDashboard.putNumber("Drive Odometry X", pose.getTranslation().getX());
      SmartDashboard.putNumber("Drive Odometry Y", pose.getTranslation().getY());
      SmartDashboard.putNumber("Drive Odometry Theta", pose.getRotation().getDegrees());

      // using vision to update odometry
      SmartDashboard.putBoolean("Vision Updating Odometry", useVisionForOdometry);

      // Values from each swerve module
      swerveFrontLeft.updateShuffleboard();
      swerveFrontRight.updateShuffleboard();
      swerveBackLeft.updateShuffleboard();
      swerveBackRight.updateShuffleboard();

      // Values for bugfixing
      SmartDashboard.putNumber("Drive Bus Volt", swerveFrontLeft.getDriveBusVoltage());
    }

    // save current angVel values as previous values for next calculation
    // prevAng = currAng;
    // prevTime = currTime; 
  }

  /**
   * Writes information about the drive train to the filelog
   * @param logWhenDisabled true will log when disabled, false will discard the string
   */
  public void updateDriveLog(boolean logWhenDisabled) {
    Pose2d pose = poseEstimator.getEstimatedPosition();
    ChassisSpeeds robotSpeeds = getRobotSpeeds();
    log.writeLog(logWhenDisabled, "Drive", "Update Variables", 
      "Gyro Angle", getGyroRotation(), "RawGyro", getGyroRaw(), 
      "Gyro Velocity", getAngularVelocity(), "Pitch", getGyroPitch(), 
      "Odometry X", pose.getTranslation().getX(), "Odometry Y", pose.getTranslation().getY(), 
      "Odometry Theta", pose.getRotation().getDegrees(),
      "Drive Speed", speedAvg,
      "Drive X Velocity", robotSpeeds.vxMetersPerSecond, 
      "Drive Y Velocity", robotSpeeds.vyMetersPerSecond,
      "Bus voltage", swerveFrontLeft.getDriveBusVoltage(),
      swerveFrontLeft.getLogString(),
      swerveFrontRight.getLogString(),
      swerveBackLeft.getLogString(),
      swerveBackRight.getLogString()
      );
  }

  public void updateOdometry() {

    poseEstimator.update(Rotation2d.fromDegrees(getGyroRotation()), getModulePositions());

    if (camera.hasInit()) {
      PhotonPipelineResult latestResult = camera.getLatestResult();
      if (latestResult != null) {
        PhotonPipelineResult camResult = latestResult;
        Optional<EstimatedRobotPose> result = camera.getEstimatedGlobalPose(poseEstimator.getEstimatedPosition(), camResult);
        if (result.isPresent()) {
          EstimatedRobotPose camPose = result.get();
          SmartDashboard.putNumber("Vision X", camPose.estimatedPose.toPose2d().getX());
          SmartDashboard.putNumber("Vision Y", camPose.estimatedPose.toPose2d().getY());
          SmartDashboard.putNumber("Vision rot", camPose.estimatedPose.toPose2d().getRotation().getDegrees());

          // Only run camera updates for pose estimator if useVisionForOdometry is true
          if (camResult.hasTargets() && useVisionForOdometry) {
            PhotonTrackedTarget bestTarget = camResult.getBestTarget();
            if (bestTarget.getBestCameraToTarget().getX() < 3) {
              poseEstimator.addVisionMeasurement(camPose.estimatedPose.toPose2d(), camPose.timestampSeconds);

              // poseEstimator.addVisionMeasurement(camPose.estimatedPose.toPose2d(), camPose.timestampSeconds, closeMatrix);
              //field.getObject("Vision").setPose(camPose.estimatedPose.toPose2d());
            }
            else if (bestTarget.getBestCameraToTarget().getX() < 7) {
                poseEstimator.addVisionMeasurement(camPose.estimatedPose.toPose2d(), camPose.timestampSeconds, farMatrix);
            }
          }
        }
      }
    }

    // Update robot average speed
    ChassisSpeeds robotSpeeds = getRobotSpeeds();
    speedAvg = Math.hypot(robotSpeeds.vyMetersPerSecond, robotSpeeds.vxMetersPerSecond);

    // Place robot on field object
    if (camera.getAlliance() == Alliance.Red) {
      Pose2d currPose = poseEstimator.getEstimatedPosition();
      double x = FieldConstants.length - currPose.getX();
      double y = FieldConstants.width - currPose.getY();
      Rotation2d rot = currPose.getRotation().rotateBy(Rotation2d.fromDegrees(180));

      field.setRobotPose(new Pose2d(x, y, rot));
    } else field.setRobotPose(poseEstimator.getEstimatedPosition());
  }  

  public void cameraInit() {
    camera.init();
    noteCamera.init();
  }

  /**
   * 
   * @param enabled true = uses vision for odometry, false = does not use vision for odometry
   */
  public void setVisionForOdometryState(boolean enabled) {
    useVisionForOdometry = enabled;
  }

  public NotePhotonCameraWrapper getNoteCamera() {
    return noteCamera;
  }

  /**
   * Returns the best target in this pipeline result. If there are no targets, this method will
   * return null. The best target is determined by the target sort mode in the PhotonVision UI.
   *
   * @return The best target of the pipeline result.
   */
  public PhotonTrackedTarget getBestTarget() {
    return noteCamera.getBestTarget();
  }

  /**
   * Returns the last photon pipeline result. If there are no new 
   * pipeline results since last call, this method will return null.
   * 
   * @return The latest pipeline result
   */
  public PhotonPipelineResult getLatestResult() {
    return noteCamera.getLatestResult();
  }

  /**
   * Turns on or off robot "aim rotation lock" on the target (speaker or long pass target)
   * for DriveWithJoysticksAdvance.
   * @param state true = aim rotation lock on, false = aim rotation lock off
   */
  public void setAimLock(boolean state) {
    aimLock = state;
  }

  /**
   * Toggles on or off robot "aim rotation lock" on the target (speaker or long pass target)
   * for DriveWithJoysticksAdvance.
   */
  public void toggleAimLock() {
    aimLock = !aimLock;
  }

  /**
   * Returns if robot "aim rotation lock" for the target (speaker or long pass target)
   * for DriveWithJoysticksAdvance is turned on or turned off
   * @return true = aim rotation lock is on, false = aim rotation lock is off
   */
  public boolean isAimLockEnabled(){
    return aimLock;
  }
}
