// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.CoordType;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.StopType;
import frc.robot.Constants.SwerveConstants;
import frc.robot.Constants.TrajectoryConstants;
import frc.robot.Constants.WristConstants.WristAngle;
import frc.robot.commands.*;
import frc.robot.commands.Autos.*;
import frc.robot.commands.Sequences.*;
import frc.robot.commands.ShooterSetVelocity.VelocityType;
import frc.robot.subsystems.*;
import frc.robot.utilities.*;
import frc.robot.utilities.BCRRobotState.ShotMode;
import frc.robot.utilities.BCRRobotState.State;
import static edu.wpi.first.wpilibj2.command.Commands.*;

import choreo.auto.AutoFactory;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Define robot key utilities (DO THIS FIRST)
  private final FileLog log = new FileLog("F10");
  private final AllianceSelection allianceSelection = new AllianceSelection(log);
  private final Timer matchTimer = new Timer();

  // Define robot subsystems  
  private final DriveTrain driveTrain = new DriveTrain(allianceSelection, log);
  private final Intake intake = new Intake("Intake", log);
  private final Shooter shooter = new Shooter(log);
  private final Feeder feeder = new Feeder(log);
  private final Wrist wrist = new Wrist(log);

  // Define other utilities
  private final TrajectoryCache trajectoryCache = new TrajectoryCache(log);
  private final AutoSelection autoSelection = new AutoSelection(trajectoryCache, allianceSelection, log);
  private final BCRRobotState robotState = new BCRRobotState();
  private final AutoFactory autoFactory; 
  
  // Is a subsystem, but requires a utility
  private final LED led = new LED(Constants.Ports.CANdle1, "LED", shooter, feeder, robotState, matchTimer, wrist, log);


  // Define controllers
  // private final Joystick xboxController = new Joystick(OIConstants.usbXboxController); //assuming usbxboxcontroller is int
  private final Joystick leftJoystick = new Joystick(OIConstants.usbLeftJoystick);
  private final Joystick rightJoystick = new Joystick(OIConstants.usbRightJoystick);
  private final Joystick coPanel = new Joystick(OIConstants.usbCoPanel);

  private final CommandXboxController xboxController = new CommandXboxController(OIConstants.usbXboxController);

  // Set to this pattern when the robot is disabled
  
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    configureButtonBindings();
    configureShuffleboard();
    autoFactory = new AutoFactory(
      driveTrain::getPose,
      driveTrain::resetPose,
      driveTrain.choreoFollowTrajectory(),
      false, 
      driveTrain);
    autoFactory.cache().loadTrajectory("Test-Path");
    // driveTrain.setDefaultCommand(new DriveWithJoystick(leftJoystick, rightJoystick, driveTrain, log));
    driveTrain.setDefaultCommand(new DriveWithJoysticksAdvance(leftJoystick, rightJoystick, allianceSelection, driveTrain, robotState, log));

  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    configureXboxButtons(); // configure xbox controller
    configureJoystickButtons(); // configure joysticks
    configureCopanel(); // configure copanel
    configureTriggers();
  }

  /**
   * Configures Shuffleboard for the robot
   */
  private void configureShuffleboard() {
    // display sticky faults
    RobotPreferences.showStickyFaultsOnShuffleboard();
    SmartDashboard.putData("Clear Sticky Faults", new StickyFaultsClear(log));

    // Intake commands
    SmartDashboard.putData("Intake Set Percent", new IntakeSetPercent(intake, log));
    SmartDashboard.putData("Intake Stop", new IntakeStop(intake, log));

    // Shooter commands
    SmartDashboard.putData("Shooter Set Percent", new ShooterSetPercent(shooter, log));
    SmartDashboard.putData("Shooter Set Velocity", new ShooterSetVelocity(VelocityType.immediatelyEnd, shooter, log));
    SmartDashboard.putData("Shooter Calibration", new ShooterCalibrationRamp(shooter, log));
    SmartDashboard.putData("ShooterFeeder Stop", new ShooterFeederStop(shooter, feeder, log));

    // Feeder commands
    SmartDashboard.putData("Feeder Set Percent", new FeederSetPercent(feeder, log));
    SmartDashboard.putData("Feeder Stop", new FeederSetPercent(0.0, feeder, log));

    // Wrist commands
    SmartDashboard.putData("Wrist Set Percent", new WristSetPercentOutput(wrist, log));
    SmartDashboard.putData("Wrist Set Angle", new WristSetAngle(wrist, log));
    SmartDashboard.putData("Wrist Calibration", new WristCalibrationRamp(0.01, 0.4, wrist, log));
    SmartDashboard.putData("Wrist Stop", new WristSetPercentOutput(0.0, wrist, log));
    SmartDashboard.putData("Wrist Nudge Angle", new WristNudgeAngle(wrist, log));
  
    // Drive base commands
    SmartDashboard.putData("Drive Toggle Coast", new DriveToggleCoastMode(driveTrain, log));
    SmartDashboard.putData("Drive Reset Pose", new DriveResetPose(driveTrain, log));
    SmartDashboard.putData("Drive To Pose", new DriveToPose(driveTrain, log));
    SmartDashboard.putData("Drive 6m +X", new DriveToPose(
      () -> driveTrain.getPose().plus(new Transform2d(6.0, 0.0, new Rotation2d(0.0))), 
      SwerveConstants.kNominalSpeedMetersPerSecond, SwerveConstants.kNominalAccelerationMetersPerSecondSquare, 
      TrajectoryConstants.maxPositionErrorMeters, TrajectoryConstants.maxThetaErrorDegrees, 
      false, false, driveTrain, log) );

    SmartDashboard.putData("Drive Calibration", new DriveCalibration(0.5, 5.0, 0.1, driveTrain, log));
    SmartDashboard.putData("Drive Turn Calibration", new DriveTurnCalibration(0.2, 5.0, 0.2 / 5.0, driveTrain, log));
    SmartDashboard.putData("Drive Percent Speed", new DrivePercentSpeed(driveTrain, log));

    SmartDashboard.putData("Drive Curve Test", new DriveTrajectory(CoordType.kRelative, StopType.kBrake, trajectoryCache.cache[TrajectoryCache.TrajectoryType.test.value].blue, driveTrain, log));
    SmartDashboard.putData("Drive Straight", new DriveStraight(false, false, false, driveTrain, log));

    // Sequences
    SmartDashboard.putData("Intake Piece", new IntakePiece(intake, feeder, wrist, shooter, robotState, log));
    SmartDashboard.putData("Shoot Piece", new ShootPiece(ShooterConstants.shooterVelocityTop, ShooterConstants.shooterVelocityBottom, true, shooter, feeder, wrist, robotState, log));
    SmartDashboard.putData("Stop All", new StopIntakeFeederShooter(intake, shooter, feeder, robotState, log));

    // Autos
    SmartDashboard.putData("Amp Two Piece Shoot", new AmpTwoPieceShoot(intake, shooter, driveTrain, feeder, wrist, robotState, trajectoryCache, allianceSelection, log));
    SmartDashboard.putData("Center Two Piece Shoot", new CenterTwoPieceShoot(intake, wrist, shooter, driveTrain, feeder, robotState, trajectoryCache, allianceSelection, log));
    SmartDashboard.putData("Source Three Piece Shoot", new SourceThreePieceShoot(intake, shooter, driveTrain, feeder, wrist, robotState, trajectoryCache, allianceSelection, log));
    SmartDashboard.putData("Source Two Piece Shoot", new SourceTwoPieceShoot(intake, wrist, shooter, driveTrain, feeder, robotState, trajectoryCache, allianceSelection, log));
    SmartDashboard.putData("Amp Shoot One Piece", new AmpShootOnePiece(intake, wrist, shooter, driveTrain, feeder, robotState, allianceSelection, log));
    SmartDashboard.putData("Source Shoot One Piece", new SourceShootOnePiece(intake, wrist, shooter, driveTrain, feeder, robotState, allianceSelection, log));

    SmartDashboard.putData("Amp Source Three Piece Shoot", new AmpSourceThreePieceShoot(intake, shooter, driveTrain, feeder, wrist, robotState, trajectoryCache, allianceSelection, log));
    SmartDashboard.putData("Source Center Three Piece Shoot", new CenterThreePieceShoot(intake, wrist, shooter, driveTrain, feeder, robotState, trajectoryCache, allianceSelection, log));
    SmartDashboard.putData("Source Start to near note",  new DriveTrajectory(CoordType.kAbsoluteResetPose, StopType.kCoast, trajectoryCache.cache[TrajectoryCache.TrajectoryType.driveToSourceCloseNote.value].red, driveTrain, log));
    SmartDashboard.putData("Drive to far note", new DriveTrajectory(CoordType.kAbsoluteResetPose, StopType.kCoast, trajectoryCache.cache[TrajectoryCache.TrajectoryType.driveAmpNoteToFarNote.value].red, driveTrain, log));

    // Copanel buttons
    SmartDashboard.putData("Climb Start", new ClimbStart(wrist, log, led));
    SmartDashboard.putData("Climb End", new ClimbEnd(wrist, log, led));
    SmartDashboard.putData("Nudge Angle Down 1 deg", new WristNudgeAngle(1, wrist, log));
    SmartDashboard.putData("Nudge Angle Up 1 deg", new WristNudgeAngle(-1, wrist, log));

    // Vision
    SmartDashboard.putData("Enable Using Vision for Odometry", new VisionOdometryStateSet(true, driveTrain, log));
    SmartDashboard.putData("Disable Using Vision for Odometry", new VisionOdometryStateSet(false, driveTrain, log));
  }

  /**
   * Creates triggers needed for the robot.
   */
  private void configureTriggers(){
    // Trigger to turn off intaking when a piece is detected in the feeder.
    // Note that this trigger will only turn off intaking if the robot is
    // currently in the INTAKING state; otherwise, it does nothing.
    Trigger intakeStopTrigger = new Trigger(()-> DriverStation.isTeleopEnabled() && 
      robotState.getState() == State.INTAKING && feeder.isPiecePresent());
    intakeStopTrigger.onTrue(
      new StopIntakingSequence(feeder, intake, robotState, log)
    );
  }

  /**
   * Configures XBox buttons and controls
   */
  private void configureXboxButtons(){
    //check povtrigger and axis trigger number bindings
    
    // Triggers for all xbox buttons
  
    Trigger xbLT = xboxController.leftTrigger();
    Trigger xbRT = xboxController.rightTrigger();
    Trigger xbA = xboxController.a();
    Trigger xbB = xboxController.b();
    Trigger xbY = xboxController.y();
    Trigger xbX = xboxController.x();
    Trigger xbLB = xboxController.leftBumper();     //empty
    Trigger xbRB = xboxController.rightBumper();
    Trigger xbBack = xboxController.back();
    Trigger xbStart = xboxController.start();
    Trigger xbPOVUp = xboxController.povUp();
    Trigger xbPOVRight = xboxController.povRight(); //empty
    Trigger xbPOVLeft = xboxController.povLeft();   //empty
    Trigger xbPOVDown = xboxController.povDown();
    Trigger xbRJoystickTrigger = xboxController.rightStick();


    
    // Prep for amp
    xbRB.onTrue( new ParallelCommandGroup(
        new IntakeStop(intake, log),
        new WristSetAngle(true, wrist, log),
        new ShotModeSet(ShotMode.AMP, robotState, log),
        new RobotStateSetIdle(robotState, feeder, log)
    ) );

    // Move wrist down and then intake a piece
    xbRT.onTrue(new IntakePiece(intake, feeder, wrist, shooter, robotState, log));

    // Reverse the intake
    xbLT.onTrue(new IntakeSetPercent(-.3, -.3, intake, log));

    // Prep for at-speaker shot
    xbA.onTrue(new SetShooterWristSpeaker(WristAngle.speakerShotFromSpeaker, 
      ShooterConstants.shooterVelocityTop, ShooterConstants.shooterVelocityBottom, shooter, wrist, intake, feeder, robotState, log));
    
    // Prep for podium speaker shot
    xbB.onTrue(new SetShooterWristSpeaker(WristAngle.speakerShotFromPodium, 
      ShooterConstants.shooterVelocityTop, ShooterConstants.shooterVelocityBottom, shooter, wrist, intake, feeder, robotState, log));
    
    // Prep for overhead speaker shot
    xbY.onTrue(new SetShooterWristSpeaker(WristAngle.overheadShotAngle, 
    ShooterConstants.shooterVelocityTop, ShooterConstants.shooterVelocityBottom, shooter, wrist, intake, feeder, robotState, log));

    // Prep for short pass
    xbPOVDown.onTrue(new SetShooterFarShot(WristAngle.shortPassAngle, 
      ShooterConstants.shooterVelocityShortPassTop, ShooterConstants.shooterVelocityShortPassBottom, shooter, wrist, intake, feeder, ShotMode.SHORT_PASS, robotState, log));

    // Prep for long pass
    xbPOVUp.onTrue(new SetShooterFarShot(WristAngle.longPassAngle, 
    ShooterConstants.shooterVelocityFarPassTop, ShooterConstants.shooterVelocityFarPassBottom, shooter, wrist, intake, feeder, ShotMode.FAR_PASS, robotState, log));

    
    // Store wrist, does not turn on intake
    xbX.onTrue(
      new ParallelCommandGroup(
        new WristLowerSafe(WristAngle.lowerLimit, feeder, wrist, log),
        new ShotModeSet(ShotMode.SPEAKER, robotState, log)
      ));
    
    // Prep for pit shot when back button is pressed
    xbBack.onTrue(new SetShooterWristSpeaker(WristAngle.lowerLimit, 
      ShooterConstants.shooterVelocityPit, ShooterConstants.shooterVelocityPit, shooter, wrist, intake, feeder, robotState, log));
    // Shoot in slow speed pit shot when released
    xbBack.onFalse( new ShootPiece( ShooterConstants.shooterVelocityPit, ShooterConstants.shooterVelocityPit, true,
      shooter, feeder, wrist, robotState, log) );

 

    // Stop all motors
    xbStart.onTrue(new ParallelCommandGroup(
        new IntakeStop(intake, log),
        new ShooterFeederStop(shooter, feeder, log),
        new RobotStateSetIdle(robotState, feeder, log)      
    ) );
    
    xbRJoystickTrigger.whileTrue(new WristXboxControl(xboxController, wrist, intake, feeder, log));     
  }

  /**
   * Define drivers joystick button mappings.
   */
  public void configureJoystickButtons() {
    JoystickButton[] left = new JoystickButton[3];
    JoystickButton[] right = new JoystickButton[3];

    for (int i = 1; i < left.length; i++) {
      left[i] = new JoystickButton(leftJoystick, i);
      right[i] = new JoystickButton(rightJoystick, i);
    }

    // Auto Drive to Amp
    left[1].whileTrue(
      new DriveToAmp(allianceSelection, intake, feeder, wrist, driveTrain, robotState, log)
    );

    // Shoot the note
    left[2].onTrue(
      new ShootFullSequence(allianceSelection, driveTrain, shooter, feeder, wrist, robotState, log)
    );

    // Right button 1:  Aim lock on speaker or midfield pass depending on location of robot on button press
    right[1].whileTrue(
      either(
        // Aim lock on speaker
        parallel(
          new SetAimLock(driveTrain, true, log),
          new ShotModeSet(ShotMode.SPEAKER, robotState, log),
          new WristSetAngleWithVision(wrist, allianceSelection, driveTrain, log),
          new ShooterSetVelocity(ShooterConstants.shooterVelocityTop, ShooterConstants.shooterVelocityBottom, VelocityType.waitForVelocity, shooter, log).withTimeout(1.5)
        ),
        // Aim lock on midfield pass
        parallel(
          new SetAimLock(driveTrain, true, log),
          new SetShooterFarShot(WristAngle.longPassAngle, 
        ShooterConstants.shooterVelocityFarPassTop, ShooterConstants.shooterVelocityFarPassBottom, 
        shooter, wrist, intake, feeder, ShotMode.VISION_MID_PASS, robotState, log)
        ),
        () -> driveTrain.getPose().getX() < FieldConstants.xThresholdMidPass
      )
      
    );
    right[1].onFalse(
      new SetAimLock(driveTrain, false, log)
    );

    // Right button 2:  Aim lock on far pass target
    right[2].whileTrue(new ParallelCommandGroup(
      new SetAimLock(driveTrain, true, log),
      new SetShooterFarShot(WristAngle.longPassAngle, 
        ShooterConstants.shooterVelocityFarPassTop, ShooterConstants.shooterVelocityFarPassBottom, 
        shooter, wrist, intake, feeder, ShotMode.VISION_FAR_PASS, robotState, log)
    ));
    right[2].onFalse(
      new SetAimLock(driveTrain, false, log)
    );
  }

  /**
   * Define Copanel button mappings.
   *  
   *  1  3  5  8
   *  2  4  6  8
   *      
   *  9  11 13 7
   *  10 12 14 7
   * 
   *  15
   *  16
   */
  public void configureCopanel() {
    JoystickButton[] coP = new JoystickButton[20];

    for (int i = 1; i < coP.length; i++) {
      coP[i] = new JoystickButton(coPanel, i);
    }

    // top row UP then DOWN, from LEFT to RIGHT
    coP[1].onTrue(new ClimbStart(wrist, log, led));
    coP[3].onTrue(new ClimbEnd(wrist, log, led));
    // Nudge angle up or down
    coP[5].onTrue(new WristNudgeAngle(-1, wrist, log)); // Nudge up
    coP[6].onTrue(new WristNudgeAngle(1, wrist, log)); // Nudge down

    coP[9].onTrue(new WristNudgeAmpAngle(1, wrist, log)); //Nudge down
    coP[10].onTrue(new WristNudgeAmpAngle(-1 ,wrist, log)); //Nudge up

    coP[11].onTrue(new DriveResetPose(0, false, driveTrain, log));
  }



  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoSelection.getAutoCommand(intake, wrist, shooter, feeder, driveTrain, robotState, autoFactory, log);
  }


  /**
   * Method called when robot is initialized.
   */
  public void robotInit() {
    SmartDashboard.putBoolean("RobotPrefs Initialized", RobotPreferences.prefsExist());
    if(!RobotPreferences.prefsExist()) {
      RobotPreferences.recordStickyFaults("RobotPreferences", log);
    }

    // compressor.disable();

    // Set initial robot position on field
    // This takes place a while after the drivetrain is created, so after any CanBus delays.
    driveTrain.resetPose(new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(0)));  
  }

  /**
   * robotPeriodic is run every 20msec
   */
  public void robotPeriodic(){
    log.advanceLogRotation();
    allianceSelection.periodic();
  }

  /**
   * Method called when robot is disabled.
   */
  public void disabledInit() {
    log.writeLogEcho(true, "Disabled", "Robot disabled");   // Don't log the word "Init" here -- it affects the Excel macro

    driveTrain.stopMotors();                // SAFETY:  Turn off any closed loop control that may be running, so the robot does not move when re-enabled.
    driveTrain.enableFastLogging(false);    // Turn off fast logging, in case it was left on from auto mode
    driveTrain.setVisionForOdometryState(true);

    matchTimer.stop();
  }

  /**
   * Method called once every scheduler cycle when robot is disabled.
   */
  public void disabledPeriodic() {
    // Set robot state
    robotState.setState(State.IDLE);

    // Check for CAN bus error.  This is to prevent the issue that caused us to be eliminated in 2020!
    if (driveTrain.canBusError()) {
      RobotPreferences.recordStickyFaults("CAN Bus", log);
    }
    // boolean error = true;  
    // if (error == false) {
    //   if(!patternTeamMoving.isScheduled()) patternTeamMoving.schedule();
    // }
    // else {
    //   patternTeamMoving.cancel();
    //   led.setStrip("Red", 0.5, 0);
    // }
  }
  
  /**
   * Method called when auto mode is initialized/enabled.
   */
  public void autonomousInit() {
    log.writeLogEcho(true, "Auto", "Mode Init");

    driveTrain.setDriveModeCoast(false);
    driveTrain.setVisionForOdometryState(false);

    // NOTE:  Do NOT reset the gyro or encoder here!!!!!
    // The first command in auto mode initializes before this code is run, and
    // it will read the gyro/encoder before the reset goes into effect.
  }

  /**
   * Method called once every scheduler cycle when auto mode is initialized/enabled
   */
  public void autonomousPeriodic() {
  }

  /**
   * Method called when teleop mode is initialized/enabled.
   */
  public void teleopInit() {
    log.writeLogEcho(true, "Teleop", "Mode Init");

    driveTrain.setDriveModeCoast(false);
    driveTrain.enableFastLogging(false);    // Turn off fast logging, in case it was left on from auto mode
    driveTrain.setVisionForOdometryState(true);

    // Set robot state
    robotState.setState(State.IDLE);

    matchTimer.reset();
    matchTimer.start();
  }

  /**
   * Method called once every scheduler cycle when teleop mode is initialized/enabled.
   */
  public void teleopPeriodic() {

  }
}