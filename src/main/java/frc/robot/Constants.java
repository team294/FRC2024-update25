// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;


import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.utilities.TrapezoidProfileBCR;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    public enum CoordType {
        kRelative,              // Relative to current robot location/facing
        kAbsolute,              // Absolute field coordinates, don't reset robot pose
        kAbsoluteResetPose,     // Absolute field coordinates, reset robot pose always
        kAbsoluteResetPoseTol;  // Absolute field coordinates, reset robot pose if robot is not close to specified position
    }

    /**
     * Options to select driving stopping types.
     */
    public enum StopType {
        kNoStop,
        kCoast,
        kBrake;
    }

    public static final class Ports{
      // public static final int CANPneumaticHub = 1;

      public static final String CANivoreBus = "CANivore";

      public static final int CANDriveFrontLeftMotor = 1;
      public static final int CANDriveFrontRightMotor = 2;
      public static final int CANDriveBackLeftMotor = 3;
      public static final int CANDriveBackRightMotor = 4;

      public static final int CANDriveTurnFrontLeftMotor = 5;
      public static final int CANDriveTurnFrontRightMotor = 6;
      public static final int CANDriveTurnBackLeftMotor = 7;
      public static final int CANDriveTurnBackRightMotor = 8;

      // Note:  Remote sensors accessed by a Talon FX (Falcon 500) must have a CAN ID of 15 or less. See errata
      // in CTRE documentation "Talon FX Remote Filter Device ID Must be 15 or Less" for more details.
      // This applies to the turn encoders, which are used as remote sensors for the turn motors.
      public static final int CANTurnEncoderFrontLeft = 9;
      public static final int CANTurnEncoderFrontRight = 10;
      public static final int CANTurnEncoderBackLeft = 11;
      public static final int CANTurnEncoderBackRight = 12;

      public static final int CANShooterTop = 13;
      public static final int CANShooterBottom = 14;

      public static final int CANFeeder = 15;

      public static final int CANIntake = 16;
      public static final int CANCenteringMotor = 17;

      public static final int CANPigeonGyro = 18;

      public static final int CANWrist1 = 19;
      public static final int CANWrist2 = 20;

      public static final int CANdle1 = 21;

      // Digital IO ports
      public static final int DIOFeederPieceSensor = 0;
      public static final int DIOWristRevThroughBoreEncoder = 4;
      public static final int DIOWristLowerLimit1 = 2;
      public static final int DIOWristLowerLimit2 = 3;
      public static final int DIOIntakePieceSensor = 9;
    }

    public static final class OIConstants {
      //Ports from last year
      public static final int usbXboxController = 0;
      public static final int usbLeftJoystick = 1;
      public static final int usbRightJoystick = 2;
      public static final int usbCoPanel = 3;

      public static final double joystickDeadband = 0.01;
      public static final double manualElevatorDeadband = 0.1;
      public static final double manualWristDeadband = 0.1;
    }

    public static final class RobotDimensions {
      // Drivebase adjustment for path-of-wheel diameter when turning in place
      private static final double DrivetrainAdjustmentFactor = 1.011;       // 1.011 CALIBRATED
      // left to right distance between the drivetrain wheels; should be measured from center to center
      public static final double DRIVETRAIN_TRACKWIDTH_METERS = 0.61595 * DrivetrainAdjustmentFactor;      // 0.61595m CALIBRATED.  Competition bot CAD = 24.25" = 0.61595m. 80% bot CAD = 0.60325m, calibrated = 0.626m.
      // front-back distance between the drivetrain wheels; should be measured from center to center
      public static final double DRIVETRAIN_WHEELBASE_METERS = 0.52705 * DrivetrainAdjustmentFactor;       // 0.52705m CALIBRATED.  Competition bot CAD = 20.75" = 0.52705m.  80% bot CAD = 0.60325m, calibrated = 0.626m.

      // distance from the ground to the center of the wrist joint (m)
      public static final double heightFromGroundToWristPivot = 0.5762625;

      // distance from the center of the wrist pivot to the intersection of the path the notes travel through the shooter (m)
      public static final double lengthOfArmFromWristPivotToCenterPathOfShooter = 0.365125;

      // Width of robot in meters
      public static final double robotWidth = 0.9144;
    }

    public static final class SwerveConstants {
        // Encoder calibration to meters travelled or wheel facing degrees
      public static final double kEncoderCPR = 1.0;                // CALIBRATED = 1.0.  Encoder counts per revolution of motor pinion gear
      public static final double kDriveGearRatio = (5.903 / 1.0);      // 2024=Modified L2.  CALIBRATED.   Mk4i = 8.14:1 (L1-std gears), 6.75:1 (L2-fast gears), 5.903 (modified L2 16-tooth gear).  
      public static final double kTurningGearRatio = (150.0/7.0 / 1.0); // CALIBRATED = 150.0/7.0.  Mk4i = 150/7 : 1
      public static final double kWheelDiameterMeters = 0.1003 * 1.020; // E2:  1.020 adjustment for new wheels CALIBRATED.  Colson wheel = nominal 4" diameter, actual 3.95" = 0.1003m.
      public static final double kDriveEncoderMetersPerTick = (kWheelDiameterMeters * Math.PI) / kEncoderCPR / kDriveGearRatio;
      public static final double kTurningEncoderDegreesPerTick = 360.0/kEncoderCPR / kTurningGearRatio;
      
      // Robot calibration for feed-forward and max speeds
      public static final double voltageCompSaturation = 12.0;
      // Max speed is used to keep each motor from maxing out, which preserves ratio between motors 
      // and ensures that the robot travels in the requested direction.  So, use min value of all 4 motors,
      // and further derate (initial test by 5%) to account for some battery droop under heavy loads.
      // Max speed measured values 3/18/2024:  All 4 motors are 4.17, 4.08, 4.2, 4.09 meters/sec.  So use 4.0 as a conservative value
      public static final double kMaxSpeedMetersPerSecond = 4.5;          // A8:  Increased from 4.0 to 4.5.  CALIBRATED
      public static final double kFullSpeedMetersPerSecond = 0.95*kMaxSpeedMetersPerSecond;  // A8:  Increased back to 0.95
      public static final double kNominalSpeedMetersPerSecond = 0.5*kMaxSpeedMetersPerSecond;
      // Max acceleration measured 3/18/2024 (with full robot weight):  7.6 - 8.4 m/sec^2.  Keep value at 7.5.
      public static final double kMaxAccelerationMetersPerSecondSquare = 7.5; // CALIBRATED
      public static final double kFullAccelerationMetersPerSecondSquare = 0.9 * kMaxAccelerationMetersPerSecondSquare;
      public static final double kNominalAccelerationMetersPerSecondSquare = 3.5; // TODO value from last year
      public static final double kMaxTurningRadiansPerSecond = 11.0;  // TODO NOT CALIBRATED
      public static final double kNominalTurningRadiansPerSecond = Math.PI;
      public static final double kMaxAngularAccelerationRadiansPerSecondSquared = 35.0;            // TODO NOT CALIBRATED - not used in code currently
      public static final double kNominalAngularAccelerationRadiansPerSecondSquared = Math.PI;

      public static final double kVDriveAvg = 0.1740 * voltageCompSaturation; // 0.1740  CALIBRATED.  0.2034 from 2023 robot.  In voltage per meters per second.
      private static final double kVmFLrel = 1.0182;      // init cal 1.0182.  CALIBRATED.  kV modifier for FL drive motor
      private static final double kVmFRrel = 0.9826;      // init cal 0.9826.  CALIBRATED.  kV modifier for FR drive motor
      private static final double kVmBLrel = 1.0102;      // init cal 1.0102.  CALIBRATED.  kV modifier for BL drive motor
      private static final double kVmBRrel = 0.9889;      // init cal 0.9889.  CALIBRATED.  kV modifier for BR drive motor
      // Normalize kVm constants
      private static double kVmAvg = (kVmFLrel + kVmFRrel + kVmBLrel + kVmBRrel)/4.0;
      public static final double kVmFL = kVmFLrel / kVmAvg;
      public static final double kVmFR = kVmFRrel / kVmAvg;
      public static final double kVmBL = kVmBLrel / kVmAvg;
      public static final double kVmBR = kVmBRrel / kVmAvg;

      public static final double dt = 0.02;       // Timestep for discretizing robot motion, in seconds.  Set this to the scheduler time period = 20ms.
      public static final double kADrive = 0.0 * voltageCompSaturation; // In voltage per meters per second squared.
      public static final double kADriveToPose = 0.100;  // Updated to 0.100 for A3, looks good.  CALIBRATED.  In time (seconds).
      public static final double kSDrive = 0.0080 * voltageCompSaturation; // init cal done.  formerly 0.0255, CALIBRATED.  In voltage.
    }

    public static final class DriveConstants {
      // The locations of the wheels relative to the physical center of the robot, in meters.
      // X: + = forward.  Y: + = to the left
      // The order in which you pass in the wheel locations is the same order that
      // you will receive the module states when performing inverse kinematics. It is also expected that
      // you pass in the module states in the same order when calling the forward kinematics methods.
      // 0 = FrontLeft, 1 = FrontRight, 2 = BackLeft, 3 = BackRight
      public static final SwerveDriveKinematics kDriveKinematics =
            new SwerveDriveKinematics(
                new Translation2d(RobotDimensions.DRIVETRAIN_WHEELBASE_METERS / 2, RobotDimensions.DRIVETRAIN_TRACKWIDTH_METERS / 2),
                new Translation2d(RobotDimensions.DRIVETRAIN_WHEELBASE_METERS / 2, -RobotDimensions.DRIVETRAIN_TRACKWIDTH_METERS / 2),
                new Translation2d(-RobotDimensions.DRIVETRAIN_WHEELBASE_METERS / 2, RobotDimensions.DRIVETRAIN_TRACKWIDTH_METERS / 2),
                new Translation2d(-RobotDimensions.DRIVETRAIN_WHEELBASE_METERS / 2, -RobotDimensions.DRIVETRAIN_TRACKWIDTH_METERS / 2));

      // Update the offset angles in RobotPreferences (in Shuffleboard), not in this code!
      // After updating in RobotPreferences, you will need to re-start the robot code for the changes to take effect.
      // When calibrating offset, set the wheels to zero degrees with the bevel gear facing to the right
      public static double offsetAngleFrontLeftMotor = 110.7; // 110.7
      public static double offsetAngleFrontRightMotor = 44.6; // 44.6
      public static double offsetAngleBackLeftMotor = -67.1; // -67.1
      public static double offsetAngleBackRightMotor = 152.1; // 152.1

        // Driving constants to cap acceleration
      public static final double maxAccelerationRate = 10.0;         // m/s^2
      public static final double maxAccelerationRateY = 5.0;          // m/s^2
      public static final double maxAccelerationRateAtScoreMid = 7;           // m/s^2
      public static final double maxAccelerationRateBetweenScoreMidAndHigh = 6.0;        // m/s^2
      public static final double maxAccelerationRateWithElevatorUp = 1.5;           // m/s^2
      public static final double maxRotationRateWithElevatorUp = 0.8;     // rad/sec

      public static final double kPJoystickThetaController = 3; // Theta kp value for joystick in rad/sec    
    }

    public static final class ShooterConstants {
      public static final double compensationVoltage = 12.0;
      public static final double ticksPerRevolution = 1.0;
      public static final double shooterGearRatio = 24.0/18.0;  // From CAD, shooter belt from motor to wheels = 24:18.  Turn ratio from motor pinion to shooter wheels.

      // PIDSVA for Top Shooter motor
      public static final double ShooterTopkP = 0.5;          // 0.5 CALIBRATED.  kP = (desired-output-volts) / (error-in-encoder-rps)
      public static final double ShooterTopkI = 0.00;         // kI = (desired-output-volts) / (error-in-encoder-rps * s)
      public static final double ShooterTopkD = 0.0;          // kD = (desired-output-volts) / (error-in-encoder-rps/s)
      public static final double ShooterTopkS = 0.139;        // kS = (desired-output-volts)
      public static final double ShooterTopkV = 0.1112;       // kV = (desired-output-volts) / (target-velocity-in-encoder-rps)
      public static final double ShooterTopkA = 0.0;          // kA = (desired-output-volts) / (target-accel-in-encoder-rot/sec^2)
      
      //PIDSVA for Bottom Shooter motor
      public static final double ShooterBottomkP = 0.5;       // 0.5
      public static final double ShooterBottomkI = 0.00;
      public static final double ShooterBottomkD = 0.0;
      public static final double ShooterBottomkS = 0.145;     // V; old: 0.004, new: 0.125
      public static final double ShooterBottomkV = 0.1140;    // V * s / dist; old: 0.000155, new: 0.129166,
      public static final double ShooterBottomkA = 0.0;

      /*
        Volt  RPS 
        0.77	5.9453125
        1.41	11.41796875
        2.06	16.814453125
      */

      public static final double velocityErrorTolerance = 100;
      public static final double shooterPercent = 0.25;
      public static final double shooterPercentStopQuickly = -0.02;       // Shooter speed to quickly ramp down shooter motor
      public static final double shooterVelocityTop = 4000;
      public static final double shooterVelocityBottom = 4400;
      public static final double shooterVelocityPit = 500;
      public static final double shooterVelocityShortPassTop = 2600;
      public static final double shooterVelocityShortPassBottom = 2600;
      public static final double shooterVelocityFarPassTop = 2970;
      public static final double shooterVelocityFarPassBottom = 2970;

      // Time for the shooter to ramp down at shooterPercentStopQuickly before stopping
      public static final double shooterSpinDownSeconds = 0.5;
    }

    public static final class FeederConstants {
      public static final double compensationVoltage = 12.0;
      public static final double ticksPerRevolution = 1.0;
      public static final double feederGearRatio = 12.0/36.0;    // From CAD, feeder gears from motor to wheels = 12:36 then 1:1.  Turn ratio from motor pinion to feeder wheels.
      
      //PIDSVA for Feeder
      public static final double kP = 0.5;
      public static final double kI = 0.00;
      public static final double kD = 0.0;
      public static final double kS = 0.061256; // V; old: 0.004, new: 0.125
      public static final double kV = 0.118681; // V * s / dist; old: 0.000155, new: 0.129166,
      public static final double kA = 0.0;

      public static final double feederPercent = 0.2;
      public static final double feederAmpShot = -0.3;
      public static final double feederBackPiecePercent = -0.05;   // Speed to back off note slightly after intaking
      public static final double feederBackPieceTime = 0.1;  // Time (in seconds) to back off note slightly after intaking
    }

    public static final class TrajectoryConstants {


      // Max error for robot rotation
      public static final double maxThetaErrorDegrees = 1.0;
      public static final double maxPositionErrorMeters = 0.04; // 1.6 inches

      // Max error for interim positions (not final)
      public static final double interimThetaErrorDegrees = 2.0;        
      public static final double interimPositionErrorMeters = 0.20; // 8 inches

      // Feedback terms for holonomic drive controllers

      // X-velocity controller:  kp.  Units = (meters/sec of velocity) / (meters of position error)
      public static final double kPXController = 1;

      // Y-velocity controller:  kp.  Units = (meters/sec of velocity) / (meters of position error)  
       public static final double kPYController = 1; 

      public static final double kPThetaController = 3;   // Theta-velocity controller:  kp.  Units = (rad/sec of velocity) / (radians of angle error)

      public static final TrajectoryConfig swerveTrajectoryConfig =
            new TrajectoryConfig(
                    SwerveConstants.kNominalSpeedMetersPerSecond,
                    SwerveConstants.kNominalAccelerationMetersPerSecondSquare)
                .setKinematics(DriveConstants.kDriveKinematics);

        /* Constraint for the motion profilied robot angle controller */
      public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
      new TrapezoidProfile.Constraints(
            SwerveConstants.kNominalTurningRadiansPerSecond, SwerveConstants.kNominalAngularAccelerationRadiansPerSecondSquared);

        /* Constraint for the DriveToPose motion profile for distance being travelled */
      public static final TrapezoidProfileBCR.Constraints kDriveProfileConstraints =
      new TrapezoidProfileBCR.Constraints(
            SwerveConstants.kNominalSpeedMetersPerSecond, SwerveConstants.kNominalAccelerationMetersPerSecondSquare);
    }

    public static class FieldConstants {
      public static final double length = Units.feetToMeters(54);
      public static final double width = Units.feetToMeters(27);
      public static final double yPosSpeakerRed = 2.663;
      public static final double yPosSpeakerBlue = width - yPosSpeakerRed;
      public static final double heightOfSpeaker = 2.03;
      
      // far pass positions
      public static final double yPosFarPassTargetRed = Units.inchesToMeters(32);   // F2:  Changed from 44 to 32
      public static final double xPosFarPassTargetRed = Units.feetToMeters(4.66);   // F3:  Changed from 3.66 to 4.66
      
      public static final double yPosFarPassTargetBlue = width - yPosFarPassTargetRed;
      public static final double xPosFarPassTargetBlue = xPosFarPassTargetRed;
      public static final Pose2d posAmpRed = new Pose2d(1.849, -.2, new Rotation2d(Units.degreesToRadians(90)));
      public static final Pose2d posAmpBlue = new Pose2d(1.849, width + 0.2, new Rotation2d(Units.degreesToRadians(-90)));
      public static final Pose2d posAmpRedInitial = new Pose2d(1.849, .2, new Rotation2d(Units.degreesToRadians(90)));
      public static final Pose2d posAmpBlueInitial = new Pose2d(1.849, width - .2, new Rotation2d(Units.degreesToRadians(-90)));

      // midfield pass positions
      public static final double xThresholdMidPass = Units.inchesToMeters(76.1 + 345.91); // F5 76.1 + 345.91

      public static final double yPosMidPassTargetRed = 1.3;   // F5: 1.3m
      public static final double xPosMidPassTargetRed = 7.25;   // F5: 7.25m
      
      public static final double yPosMidPassTargetBlue = width - yPosMidPassTargetRed;
      public static final double xPosMidPassTargetBlue = xPosMidPassTargetRed;
    }

    public static class VisionConstants {
      // PhotonVision
      public static class PhotonVisionConstants {
        public static final int width = 1200;
        //TODO NEED TO CALIBRATE
        public static final Transform3d robotToCamFront =
                new Transform3d(
                    // new Translation3d(Units.inchesToMeters(6.0), 0.0, Units.inchesToMeters(30.5)),       Changed in B3
                    new Translation3d(Units.inchesToMeters(8.9375), Units.inchesToMeters(0), Units.inchesToMeters(25.03125)),
                    new Rotation3d(0, Units.degreesToRadians(15), Units.degreesToRadians(180))); // Cam mounted facing forward in center of robot
        public static final String aprilTagCameraName = "AprilTagCamera";
        // 1.75 physical center to wheel center
        // 16.75 wheel cetner to intake center without bumper
        
        public static final Transform3d robotToCamBack =
                new Transform3d(
                    // new Translation3d(Units.inchesToMeters(6.0), 0.0, Units.inchesToMeters(30.5)),       Changed in B3
                    new Translation3d(Units.inchesToMeters(6.125), Units.inchesToMeters(0), Units.inchesToMeters(25.03125)),
                    new Rotation3d(0, Units.degreesToRadians(15), Units.degreesToRadians(180))); // Cam mounted facing forward in center of robot

             
        public static final String aprilTagCameraBackName = "AprilTagCameraBack";


        public static final String noteCameraName = "NoteCamera";
        public static final double pitchSetpoint = -18;
        public static final double yawSetpoint = 0;
      }
    }

    public static final class WristConstants {
      public static final double kEncoderCPR = 1.0;                // CALIBRATED = 1.  Encoder counts per revolution of FalconFX motor pinion gear
      public static final double kWristGearRatio = (5.0*5.0*3.0 * 48.0 / 22.0);   // From CAD, should be 5*5*3 * 48:22.  Gear reduction ratio between motor pinion and gear driving the wrist (planetary and chain gears)
      public static final double kWristDegreesPerRotation =  360.0 / kEncoderCPR / kWristGearRatio;      // CALIBRATED (fudge factor was 0.9726 last year)

      public static final double kRevEncoderGearRatio = (48.0 / 22.0);   // From CAD, should be 48:22.  Gear reduction ratio between Rev Thru-Bore encoder and gear driving the wrist (chain/gears)
      
      public static final double voltageCompSaturation = 12.0;
      public static final double maxUncalibratedPercentOutput = 0.15;     // CALIBRATED
      public static final double maxPercentOutput = 0.4;          // CALIBRATED

      public static final double climbPercentOutput = -0.5;

      // Update the REV through bore encoder offset angle in RobotPreferences (in Shuffleboard), not in this code!
      // After updating in RobotPreferences, you will need to re-start the robot code for the changes to take effect.
      // When calibrating offset, 0 deg should be with the CG of the wrist horizontal facing away from the robot,
      // and -90 deg is with the CG of the wrist resting downward.
      public static double revEncoderOffsetAngleWrist = 150.5;    // E3: Updated 150.5 deg (added arm spacer)  CALIBRATED

      public static final double kP = 0.5;   // 0.5 CALIBRATED.  kP = (desired-output-volts) / (error-in-encoder-rotations)
      public static final double kI = 0.0; 
      public static final double kD = 0.0; 
      public static final double kG = 0.174;   // 0.174 CALIBRATED.  Feed foward voltage to add to hold arm horizontal (0 deg)
      public static final double kS = 0.0367;  // 0.0367 CALIBRATED
      public static final double kV = 0.1171;  // 0.1171 CALIBRATED

      public static final double MMCruiseVelocity = 90.0;   // 90.0 Calibrated.  Arm can reach ~95.  Max trapezoid velocity in motor rps.
      public static final double MMAcceleration = MMCruiseVelocity/0.35;    // Calibrated.  Accel in 0.35 sec.  Max trapezoid acceleration in motor rot/sec^2.  MMVel/MMAccel = (# seconds to full velocity)
      public static final double MMJerk = MMAcceleration/0.05;  // Calibrated.  Jerk in 0.05 sec.  Max trapezoid jerk in motor rot/sec^3.  MMAccel/MMJerk = (# seconds to full accel)

      public static final double wristShootTolerance = 2.0;   // Only shoot if wrist is within this many degrees of the target angle

      // Wrist regions
      public enum WristRegion {
          // backFar,        // In the wrist backFar region, the elevator must be in the bottom region (not allowed to go to elevator main or low regions).
          // backMid,        // In the wrist backMid region, the elevator may be in any elevator region.
          // down,           // Wrist pointed down, the elevator must be in the main region.
          // back,           // In the wrist back region, the elevator must be in the bottom region (not allowed to go to elevator main).
          main,           // In the wrist main region, the elevator may be in any elevator region.
          uncalibrated    // Unknown region, wrist is not calibrated
      } 
      // Wrist region boundaries
      // public static final double boundBackFarMid = -119.0;      // Boundary between backFar and backMid regions.  CALIBRATED
      // public static final double boundBackMidDown = -116.0;      // Boundary between backMid and down regions.  CALIBRATED
      // public static final double boundDownMain = -91.0;      // Boundary between down and main regions.  CALIBRATED
      // public static final double boundDownMidpoint = (boundBackMidDown+boundDownMain)/2.0;      // Midpoint in down region
      // public static final double boundBackMain = -120.0;      // Boundary between back and main regions.  CALIBRATED

      // Wrist pre-defined angles (in degrees)
      // 0 degrees = horizontal (in front of robot) relative to wrist center of gravity
      // -90 degrees = vertical = wrist is hanging "down" naturally due to gravity
      public enum WristAngle {
          lowerLimit(-83.0),      // CALIBRATED
          intakeLimit(-75), // Max angle that we can intake from. CALIBRATED (we know -75deg is ok.  Maybe could be higher, not tested.)
          speakerShotFromSpeaker(-38),  // A5: changed to -42 deg
          speakerShotFromSide(-38),
          speakerShotFromPodium(-67),  // A4: changed to -70 deg.  Practice field -72deg for 128" field edge to front of bumper, ~144" to robot origin
          speakerShotFromMidStage(-79),
          shortPassAngle(-79),
          longPassAngle(-60),
          sourceCloseNoteShot(-61),
          centerCloseNoteShot(-62),
          ampCloseNoteShot(-63),
          endFiveNoteShot(-69),
          sourceThreePieceShot(-69),
          endAmpFourcePieceShot(-66), // last note shot for amp 4 note 
          ampFourPieceShot(-72),     // F5: Increased by 1 degree (-73 to -72)
          overheadShotAngle(57),      // D1:  Increased from 56 to 57 deg Worlds Fri before matches.
          climbStop(-45.0),
          ampShot(52.0),            // C4:  Increased from 50 to 52
          clearBellyPanMinAngle(-65),
          climbStart(80.0),       // Increased from 65 to 80
          upperLimit(90.0);       // CALIBRATED

          @SuppressWarnings({"MemberName", "PMD.SingularField"})
          public final double value;
          WristAngle(double value) { this.value = value; }
      }
    }

    public static final class IntakeConstants {
      public static final double compensationVoltage = 12.0;                      // voltage compensation on motor

      public static final double intakingPieceCurrentThreshold = 35.0;            // Current to indicate intake is loading a piece (or jammed), in Amps.  Note that the intake turning on can briefly spike this high as well.

      public static final double intakePercent = 0.7;       // 0.7
      public static final double centeringPercent = 0.4;    // 0.4
    }

    /** Colors for the LEDs based on different robot states (see BCRRobotState) */
    public enum BCRColor {
      IDLE(255, 255, 255), // White             (nothing running)
      INTAKING(0, 0, 255), // Blue       (intake running)
      SHOOTING(0, 255, 0), // Green       (shooter running)
      isRainbow(Color.kBlack),
      isPiecePresent(255, 30, 0),
      isShooterVelocityWithinError(0, 255, 0),
      isShooterRPMAboveZero(Color.kPurple),
      isWristCalibrated(255, 255, 0);

      public final int r, g, b;
      BCRColor(int r, int g, int b) {
          this.r = r;
          this.g = g;
          this.b = b;
          this.col = Color.kBlack;
      }

      public final Color col;
      BCRColor (Color c) {
        this.r = 0;
        this.g = 0;
        this.b = 0;
        this.col = c;
      }
  }

    public static final class LEDConstants {
      public static final double accuracyDisplayThreshold = 35; //TODO Decide what the threshold should be

      public static final class Patterns {
          // Static Patterns
          public static final Color[] blueOrangeStatic = {Color.kBlue, Color.kOrange};
          public static final Color[] accuracyDisplayPattern = {Color.kRed};
          // Animated Patterns
          public static final Color[][] blueOrangeMovingAnim = {{Color.kBlue, Color.kOrange},{Color.kOrange,Color.kBlue}};
          public static final Color[][] rainbowArray = {
              {Color.kRed, Color.kOrangeRed, Color.kOrange, Color.kRed, Color.kOrangeRed, Color.kOrange},
              {Color.kGreen, Color.kGreenYellow, Color.kLime, Color.kGreen, Color.kGreenYellow, Color.kLime},
              {Color.kBlue, Color.kAliceBlue, Color.kAquamarine, Color.kBlue, Color.kAliceBlue, Color.kAquamarine}
          };
          // Utilities
          public static final Color[] noPatternStatic = {};
          public static final Color[][] noPatternAnimation = {{}};
          public static final Color[] clearPatternStatic = {Color.kBlack};
      }

      public enum LEDSegmentRange {
          CANdle(0,8), // Whole CANdle
          StripLeft(32, 29),  // Left strip only -- D5:  Updated for 2 less LEDs
          StripRight(61, 30), // Right strip only -- D5:  Updated for 2 less LEDs
          StripHorizontal(8, 24); // Horizontal strip only -- D5:  Updated for 2 less LEDs

          public final int index, count;
          LEDSegmentRange(int index, int count) {
              this.index = index;
              this.count = count;
          }
      }
  }
}