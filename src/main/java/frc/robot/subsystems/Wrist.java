/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.Ports;
import frc.robot.Constants.WristConstants.WristAngle;
import frc.robot.Constants.WristConstants.WristRegion;
import frc.robot.utilities.FileLog;
import frc.robot.utilities.Loggable;
import frc.robot.utilities.MathBCR;
import frc.robot.utilities.RobotPreferences;
import frc.robot.utilities.Wait;

import static frc.robot.Constants.WristConstants.*;

public class Wrist extends SubsystemBase implements Loggable{
  private final FileLog log;
  private int logRotationKey;         // key for the logging cycle for this subsystem
  private boolean fastLogging = false;
  private final String subsystemName;
  private final Timer bootTimer = new Timer();
  
  private final TalonFX wristMotor1 = new TalonFX(Ports.CANWrist1);
  private final TalonFX wristMotor2 = new TalonFX(Ports.CANWrist2);
	private final TalonFXConfigurator wristMotor1Configurator = wristMotor1.getConfigurator();
	private TalonFXConfiguration wristMotor1Config;
  private final TalonFXConfigurator wristMotor2Configurator = wristMotor2.getConfigurator();
	private TalonFXConfiguration wristMotor2Config;
	private VoltageOut wristVoltageControl = new VoltageOut(0.0).withEnableFOC(false);
  private PositionVoltage wristPositionControl = new PositionVoltage(0.0).withEnableFOC(false);
  private MotionMagicVoltage wristMMVoltageControl = new MotionMagicVoltage(0.0).withEnableFOC(false);
  

	// Variables for motor signals and sensors
	private final StatusSignal<Temperature> wrist1Temp = wristMotor1.getDeviceTemp();				  // Motor temperature, in degC
	private final StatusSignal<ControlModeValue> wrist1ControlMode = wristMotor1.getControlMode();			// Motor control mode (typ. ControlModeValue.VoltageOut or .PositionVoltage)
	private final StatusSignal<Double> wrist1DutyCycle = wristMotor1.getDutyCycle();				  // Motor duty cycle percent power, -1 to 1
  private final StatusSignal<Voltage> wrist1MotorVotage = wristMotor1.getMotorVoltage();       // Motor output voltage
	private final StatusSignal<Current> wrist1StatorCurrent = wristMotor1.getStatorCurrent();	// Motor stator current, in amps (+=fwd, -=rev)
	private final StatusSignal<Angle> wrist1EncoderPostion = wristMotor1.getPosition();			// Encoder position, in pinion rotations
  private final StatusSignal<AngularVelocity> wrist1EncoderVelocity = wristMotor1.getVelocity();     // Encoder velocity, in pinion rotations per second
  private final StatusSignal<AngularAcceleration> wrist1EncoderAcceleration = wristMotor1.getAcceleration();     // Encoder acceleration, in pinion rotations per second^2

	private final StatusSignal<Temperature> wrist2Temp = wristMotor2.getDeviceTemp();				  // Motor temperature, in degC
	private final StatusSignal<Double> wrist2DutyCycle = wristMotor2.getDutyCycle();				  // Motor duty cycle percent power, -1 to 1
	private final StatusSignal<Current> wrist2StatorCurrent = wristMotor2.getStatorCurrent();	// Motor stator current, in amps (+=fwd, -=rev)
	private final StatusSignal<Angle> wrist2EncoderPostion = wristMotor2.getPosition();			// Encoder position, in pinion rotations

  // Wrist bump switches
  private final DigitalInput lowerLimit1 = new DigitalInput(Ports.DIOWristLowerLimit1);
  private final DigitalInput lowerLimit2 = new DigitalInput(Ports.DIOWristLowerLimit2);

  // Rev through-bore encoder
  private final DutyCycleEncoder revEncoder = new DutyCycleEncoder(Ports.DIOWristRevThroughBoreEncoder);

  private boolean calibrationStickyFaultReported = false;   // True if we have reported a sticky fault for wrist calibration
  private double revEncoderZero = 0;          // Reference raw encoder reading for encoder.  Calibration sets this to the absolute position from RobotPreferences.
  private double wristCalZero = 0;   		      // Wrist encoder position at O degrees, in degrees (i.e. the calibration factor).  Calibration sets this to match the REV through bore encoder.
  private double wristCalZero2 = 0;   		     // Wrist encoder #2 position at O degrees, in degrees (i.e. the calibration factor).  Calibration sets this to match the REV through bore encoder.
  private boolean wristCalibrated = false;    // Default to wrist being uncalibrated.  Calibrate from robot preferences or "Calibrate Wrist Zero" button on dashboard

  private double safeAngle;         // current wrist target on position control on the Falcon motor (if the Falcon is in position mode)

  private double ampAngleOffset = 0; 
  
  public Wrist(FileLog log) {
    this.log = log;
    logRotationKey = log.allocateLogRotation();     // Get log rotation for this subsystem
    subsystemName = "Wrist";

		// Start with factory default TalonFX configuration
		wristMotor1Config = new TalonFXConfiguration();			// Factory default configuration
    wristMotor2Config = new TalonFXConfiguration();	

    // Configure motor
 		wristMotor1Config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;		// Don't invert motor 1 output so that +Volt moves wrist up
		wristMotor1Config.MotorOutput.NeutralMode = NeutralModeValue.Brake;          // Applies during VoltageControl only, since setting is being overridded for PositionControl
		// wristMotorConfig.MotorOutput.DutyCycleNeutralDeadband = 0;  // Default = 0
		// wristMotorConfig.MotorOutput.PeakForwardDutyCycle = 1.0;			// Default = 1.0.  We probably won't use duty-cycle control, since there is no longer voltage compensation
		// wristMotorConfig.MotorOutput.PeakReverseDutyCycle = -1.0;			// Default = -1.0.  We probably won't use duty-cycle control, since there is no longer voltage compensation
		wristMotor1Config.Voltage.PeakForwardVoltage = voltageCompSaturation;    // forward max output for motor 1
		wristMotor1Config.Voltage.PeakReverseVoltage = -voltageCompSaturation;   // back max output for motor 1
		wristMotor1Config.OpenLoopRamps.VoltageOpenLoopRampPeriod = 0.3;		      // 0.3 seconds
		wristMotor1Config.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 0.3; 		// 0.3 seconds

    wristMotor2Config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;		// Invert motor 2 output so that +Volt moves wrist up
		wristMotor2Config.MotorOutput.NeutralMode = NeutralModeValue.Brake;          // Applies during VoltageControl only, since setting is being overridded for PositionControl
    wristMotor2Config.Voltage.PeakForwardVoltage = voltageCompSaturation;    // forward max output for motor 2
		wristMotor2Config.Voltage.PeakReverseVoltage = -voltageCompSaturation;   // back max output for motor 2
		wristMotor2Config.OpenLoopRamps.VoltageOpenLoopRampPeriod = 0.3;		      // 0.3 seconds for motor 2
		wristMotor2Config.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 0.3; 		// 0.3 seconds for motor 2

    // Configure encoder on motor 1 and 2
		wristMotor1Config.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
    wristMotor1Config.ClosedLoopGeneral.ContinuousWrap = false;
    // wristMotor1Config.Feedback.SensorToMechanismRatio = 1.0;       // Need to set for MotionMagic if using Phoenix kG!!!
    // wristMotor1Config.Feedback.FeedbackRotorOffset = 0.0;          // Need to set for MotionMagic if using Phoenix kG!!!
    wristMotor2Config.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
    wristMotor2Config.ClosedLoopGeneral.ContinuousWrap = false;

    // Configure PID for PositionVoltage control
    // Note:  In Phoenix 6, slots are selected in the ControlRequest (ex. PositionVoltage.Slot)
    wristPositionControl.Slot = 0;
    wristPositionControl.OverrideBrakeDurNeutral = true;
    wristMMVoltageControl.Slot = 0;
    wristMMVoltageControl.OverrideBrakeDurNeutral = true;
    wristMotor1Config.Slot0.kP = kP;		// kP = (desired-output-volts) / (error-in-encoder-rotations)
		wristMotor1Config.Slot0.kI = 0.0;
		wristMotor1Config.Slot0.kD = 0.0;
		wristMotor1Config.Slot0.kS = kS;
		wristMotor1Config.Slot0.kV = kV;
		wristMotor1Config.Slot0.kA = 0.0;
		// wristMotor1Config.Slot0.kG = kG;                   // We don't have a 1:1 encoder right now, so don't use Phoenix kG.  Use kG in Arbitrary Feed Forward instead
		// wristMotor1Config.Slot0.GravityType = GravityTypeValue.Arm_Cosine;       // Also see SensorToMechanismRatio and FeedbackRotorOffset above

    //set Magic Motion Settings
		wristMotor1Config.MotionMagic.MotionMagicCruiseVelocity = MMCruiseVelocity;
		wristMotor1Config.MotionMagic.MotionMagicAcceleration = MMAcceleration;
		wristMotor1Config.MotionMagic.MotionMagicJerk = MMJerk;

    // wristMotor2Config.Slot0.kP = kP;		// kP = (desired-output-volts) / (error-in-encoder-rotations)
		// wristMotor2Config.Slot0.kI = 0.0;
		// wristMotor2Config.Slot0.kD = 0.0;

 		// Apply configuration to the wrist motor 1 and 2 
		// This is a blocking call and will wait up to 50ms-70ms for the config to apply.  (initial test = 62ms delay)
		wristMotor1Configurator.apply(wristMotor1Config);
    wristMotor2Configurator.apply(wristMotor2Config);

    //Make wrist motor 2 follow motor 1 
    wristMotor2.setControl(new Follower(wristMotor1.getDeviceID(), true));     // OpposeMasterDirection=true because motors are flipped relative to each other
    
    // NOTE!!! When the TalonFX encoder settings are changed above, then the next call to 
    // getTurningEncoderDegrees() may contain an old value, not the value based on 
    // the updated configuration settings above!!!!  The CANBus runs asynchronously from this code, so 
    // sending the updated configuration to the CanCoder/TalonFX and then receiving an updated position measurement back
    // may take longer than this code.
    // The timeouts in the configuration code above should take care of this, but it does not always wait long enough.
    // So, add a wait time here:
    Wait.waitTime(250);

    stopWrist();

    // Rev Encoder takes a while to boot.  Calibrate encoder in Wrist.periodic() after it boots.
    // Set timer for calibration
    bootTimer.reset();
    bootTimer.start();
    revEncoderZero = -revEncoderOffsetAngleWrist;     // Prime the revEncoderZero from RobotPreferences
  }

  /**
   * Returns the name of the subsystem
   */
  public String getName() {
    return subsystemName;
  }

	// ************ Wrist movement methods

  /**
   * Stops both wrist motors 
   */
  public void stopWrist() {
    setWristMotorPercentOutput(0.0);
  }

  /**
   * Sets percent power of both wrist motors
   * <p><b> There are no elevator interlocks on this method!!!! </b>
   * @param percentPower between -1.0 (down full speed) and 1.0 (up full speed)
   */
  public void setWristMotorPercentOutput(double percentOutput) {
    if (wristCalibrated) {
      percentOutput = MathUtil.clamp(percentOutput, -maxPercentOutput, maxPercentOutput);
    } else {
      percentOutput = MathUtil.clamp(percentOutput, -maxUncalibratedPercentOutput, maxUncalibratedPercentOutput);
    }

    wristMotor1.setControl(wristVoltageControl.withOutput(percentOutput*voltageCompSaturation));
  }

 	/**
	 * Gets the wrist percent output of the voltage compensation limit.
	 * @return between -1.0 (down) and 1.0 (up)
	 */
	public double getWristMotorPercentOutput() {
		wrist1DutyCycle.refresh();			// Verified that this is not a blocking call.
		return wrist1DutyCycle.getValueAsDouble();
	}

  /**
   * Returns a boolean to indicate if the wrist is in position control or direct
   * percent output control.
   * @return true = position control, false = direct percent output control
   */
  public boolean isWristMotorPositionControl() {
    return (wrist1ControlMode.refresh().getValue() == ControlModeValue.PositionVoltage) || 
           (wrist1ControlMode.refresh().getValue() == ControlModeValue.MotionMagicVoltage) ||
           (wrist1ControlMode.refresh().getValue() == ControlModeValue.PositionVoltageFOC) || 
           (wrist1ControlMode.refresh().getValue() == ControlModeValue.MotionMagicVoltageFOC);
  }

  /**
   * Only works when encoder is working and calibrated
   * Interlocks with elevator position
   * @param angle target angle, in degrees (0 = horizontal in front of robot, + = up, - = down)
   */
  public void setWristAngle(double angle) {
    if (wristCalibrated) {
      // Keep wrist in usable range
      safeAngle = MathUtil.clamp(angle, WristAngle.lowerLimit.value, WristAngle.upperLimit.value);

      // WristRegion curRegion = getRegion(getWristAngle());
      // Check and apply interlocks      

      // Phoenix6 PositionVoltage control:  Position is in rotor rotations, FeedFoward is in Volts
      // wristMotor1.setControl(wristPositionControl.withPosition(wristDegreesToEncoderRotations(safeAngle))
      //                       .withFeedForward(kG * Math.cos(safeAngle*Math.PI/180.0) ));
      // Phoenix6 MotionMagicVoltage control:  Position is in rotor rotations, FeedFoward is in Volts
      wristMotor1.setControl(wristMMVoltageControl.withPosition(wristDegreesToEncoderRotations(safeAngle))
                            .withFeedForward(kG * Math.cos(safeAngle*Math.PI/180.0) ));

      log.writeLog(false, subsystemName, "Set angle", "Desired angle", angle, "Set angle", safeAngle);

      SmartDashboard.putNumber("Wrist set raw ticks", wristDegreesToEncoderRotations(safeAngle));
    }
  }

  /**
	 * Returns the angle that wrist is trying to move to in degrees.
	 * If the wrist is not calibrated, then returns wrist lowerLimit,
   * since we really don't know where the wrist is at.  If the wrist is in manual control mode, then
   * returns the actual wrist position.
	 * @return desired degree of wrist angle
	 */
  public double getCurrentWristTarget() {
    double currentTarget;

    if (wristCalibrated) {
      if (isWristMotorPositionControl()) {
        currentTarget = safeAngle;
      } else {
        // If we are not in position control mode, then we aren't moving towards a target (and the target
        // angle may be undefined).  So, get the actual wrist angle instead.
        currentTarget = getWristAngle();
      }
      return currentTarget;
    } else {
      // Wrist is not calibrated.  Assume we are at back angle in keepout region to engage all interlocks,
      // since we really don't know where the wrist is at.
      return WristAngle.lowerLimit.value;
    }
  }

	// ************ Wrist region methods

 	/**
   * For use in the wrist subsystem only.  Use getWristRegion() when calling from outside this class.
	 * <p>Returns the wrist region for a given angle.
   * @param degrees angle in degrees
	 * @return corresponding wrist region
	 */
	private WristRegion getRegion(double degrees) {
    return WristRegion.main; 
	}

	/**
	 * Returns the wrist region that the wrist is currently in.  If the wrist is moving between regions, the
   * value will return the more restrictive of the two regions.
	 * @return current wristRegion
	 */
	public WristRegion getWristRegion() {
    if (!wristCalibrated) {
      return WristRegion.uncalibrated;
    }

    WristRegion curRegion = getRegion(getWristAngle());

    // if (isWristMotorPositionControl()) {
    //   WristRegion targetRegion = getRegion(safeAngle);

    //   if (targetRegion != WristRegion.main) curRegion = WristRegion.back;
    // }
      
    return curRegion;
	}

	// ************ Internal Falcon encoder methods

  /**
   * 
   * @return raw encoder reading, in pinion rotations, adjusted direction (positive is towards stowed, negative is towards lower hard stop)
   */
  public double getWristEncoderRotationsRaw() {
    wrist1EncoderPostion.refresh();          // Verified that this is not a blocking call.
    return wrist1EncoderPostion.getValueAsDouble();
  }

  /**
   * 
   * @return raw encoder reading (2nd wrist motor), in pinion rotations, adjusted direction (positive is towards stowed, negative is towards lower hard stop)
   */
  public double getWristEncoder2RotationsRaw() {
    wrist2EncoderPostion.refresh();          // Verified that this is not a blocking call.
    return wrist2EncoderPostion.getValueAsDouble();
  }

  /**
   * Adjust the current calibration degrees of the wrist by a small amount
   * @param deltaDegrees the number of degrees to move up/down
   */
  public void nudgeWristAngle(double deltaDegrees) {
    if (!wristCalibrated) {
      // Do not attempt this if the wrist is not calibrated
      return;
    }

    // Save the current write control method (position vs voltage/off)
    boolean isPositionControl = isWristMotorPositionControl();

    // Adjust by recalibrating with a modified degrees, then set to the new angle
    // Note that calibrating the wrist turns off position control.
    calibrateWristEnc(getWristEncoderDegrees() + deltaDegrees);

    // Only set the angle if in position control mode
    if (isPositionControl) {
      setWristAngle(safeAngle);
    }
  }

  /**
   * Adjust the current variable for amp angle of the wrist by a small amount
   * @param deltaDegrees the number of degrees to move up/down
   */
  public void nudgeAmpAngle(double deltaDegrees){
    ampAngleOffset += deltaDegrees;
  }

  /**
   * Returns the current value of the amp angle shooting offset (higher or lower by
   * a few degrees).
   * @return
   */
  public double getAmpOffSet() {
    return ampAngleOffset;
  }

  /**
   * Converts the wrist position in degrees to rotations.  Assumes that the encoder is calibrated.
   * @param degrees wrist position in degrees
   * @return wrist position in rotations
   */
  private double wristDegreesToEncoderRotations(double degrees) {
    return (degrees + wristCalZero) / kWristDegreesPerRotation;
  }

  /**
   * For use in the wrist subsystem only.  Use getWristAngle() when calling from outside this class.
   * Assumes that the encoder is calibrated.
   * <p>Gets the current wrist angle from the Falcon encoder.
   * @return current encoder ticks (based on zero) converted to degrees
   */
  private double getWristEncoderDegrees() {
    // DO NOT normalize this angle.  It should not wrap, since the wrist mechanically can not cross the -180/+180 deg point
    return getWristEncoderRotationsRaw()* kWristDegreesPerRotation - wristCalZero;
  }

  /**
   * For use in the wrist subsystem only.  Use getWristAngle() when calling from outside this class.
   * Assumes that the encoder is calibrated.
   * <p>Gets the current wrist angle from the Falcon encoder.
   * @return current encoder ticks (based on zero) converted to degrees
   */
  private double getWristEncoder2Degrees() {
    // DO NOT normalize this angle.  It should not wrap, since the wrist mechanically can not cross the -180/+180 deg point
    return getWristEncoder2RotationsRaw()* kWristDegreesPerRotation - wristCalZero2;
  }

  /**
	 * Returns the angle that wrist is currently positioned at in degrees.
	 * If the wrist is not calibrated, then returns wrist lowerLimit,
   * since we really don't know where the wrist is at.
	 * @return current degree of wrist angle
	 */
  public double getWristAngle() {
    if (wristCalibrated) {
      return getWristEncoderDegrees();
    } else {
      // Wrist is not calibrated.  Assume we are at back angle in keepout region to engage all interlocks,
      // since we really don't know where the wrist is at.
      return WristAngle.lowerLimit.value;
    }
  }

  // ************ Internal Falcon encoder calibration methods

  /**
	 * returns whether encoder is calibrated or not
	 * @return true if encoder is calibrated and working, false if encoder broke
	 */
	public boolean isEncoderCalibrated() {
		return wristCalibrated;
  }

	/**
	 * Stops wrist motor and sets wristCalibrated to false
	 */
	public void setWristUncalibrated() {
		stopWrist();

    log.writeLog(false, "Wrist", "Uncalibrate wrist", 
      "Rev angle", getRevEncoderDegrees(), "Enc Raw", getWristEncoderRotationsRaw(),
			"Wrist Angle", getWristAngle(), "Wrist Target", getCurrentWristTarget());

    wristCalibrated = false;
  }

  /**
   * Calibrates the wrist encoder, assuming we know the wrist's current angle.
   * Sets wristCalibrated = true.
   * @param angle current angle that the wrist is physically at, in degrees
   */
  public void calibrateWristEnc(double angle) {
		stopWrist();	// Stop motor, so it doesn't jump to new value

    wristCalZero = getWristEncoderRotationsRaw()* kWristDegreesPerRotation - angle;
    wristCalZero2 = getWristEncoder2RotationsRaw()* kWristDegreesPerRotation - angle;
		wristCalibrated = true;

    log.writeLog(true, "Wrist", "Calibrate wrist", "zero value", wristCalZero, 
			"Rev angle", getRevEncoderDegrees(), "Enc Raw", getWristEncoderRotationsRaw(),
			"Wrist Angle", getWristAngle(), "Wrist Target", getCurrentWristTarget());
  }  
  
 	// ************ REV through-bore encoder methods

  /**
   * Calibrates the REV through bore encoder, so that 0 should be with the CG of the wrist horizontal 
   * facing away from the robot, and -90 deg is with the CG of the wrist resting downward.
   * <p> <b> NOTE!!!! </b> Only call this method when the wrist is in/near the down position!!!!
   * @param offsetDegrees Desired encoder zero angle, in absolute magnet position reading
   */
  public void calibrateRevEncoderDegrees(double offsetDegrees) {
    revEncoderZero = -offsetDegrees;
    
    // Avoid wrap point on Rev encoder.
    // Assume that the wrist is in/near the down position.  Then the Rev encoder should be between
    // 0 and 1 (rotation units).  If the encoder has wrapped, then adjust the encoder zero point
    // by the number of integer encoder rotations.
    revEncoderZero += 360.0/kRevEncoderGearRatio * Math.floor( revEncoder.get() );

    // // Fix encoder value if calibrated in "sort of up" position
    // if (getRevEncoderDegrees()< WristAngle.lowerLimit.value - 10) {
    //   revEncoderZero -= 360.0/kRevEncoderGearRatio;
    // }
    // if (getRevEncoderDegrees()> WristAngle.upperLimit.value + 10) {
    //   revEncoderZero += 360.0/kRevEncoderGearRatio;
    // }

    log.writeLogEcho(true, subsystemName, "calibrateThroughBoreEncoder", "encoderZero", revEncoderZero, 
        "raw encoder", revEncoder.get()*360.0/kRevEncoderGearRatio, "encoder degrees", getRevEncoderDegrees());
  }

  /**
   * Returns status of the Rev through bore encoder
   * @return true = encoder is connected, false = encoder not connected
   */
  public boolean isRevEncoderConnected() {
    return revEncoder.isConnected();
  }

  /**
   * @return Wrist orientation measured by the REV through bore encoder wrist facing, in degrees [-180,+180).
   * When calibrated, 0 should be with the CG of the wrist horizontal 
   * facing away from the robot, and -90 deg is with the CG of the wrist resting downward.
   */
  public double getRevEncoderDegrees() {
    // Note that rev encoder is not reversed [revEncoder.get()], since mounting of the encoder
    // is no longer flipped on the wrist axle.
    return MathBCR.normalizeAngle(revEncoder.get()*360.0/kRevEncoderGearRatio - revEncoderZero);
  }

 	// ************ Bump switch methods

  /**
   * Checks bump switch 1 to check if the wrist is at the lower limit.
   * @return true = at lower limit, false = not at lower limit
   */
  private boolean isWristAtLowerLimit1() {
    return !lowerLimit1.get();
  }

  /**
   * Checks bump switch 2 to check if the wrist is at the lower limit.
   * @return true = at lower limit, false = not at lower limit
   */
  private boolean isWristAtLowerLimit2() {
    return !lowerLimit2.get();
  }

  /**
   * Checks bump switches to check if the wrist is at the lower limit.
   * If either bump switch (or both switches) are pressed, then this method returns true.
   * @return true = at lower limit, false = not at lower limit
   */
  public boolean isWristAtLowerLimit() {
    return isWristAtLowerLimit1() || isWristAtLowerLimit2();
  }

 	// ************ Periodic and information methods

  /**
   * Writes information about the subsystem to the filelog
   * @param logWhenDisabled true will log when disabled, false will discard the string
   */
  public void updateWristLog(boolean logWhenDisabled) {
    log.writeLog(logWhenDisabled, subsystemName, "Update Variables",
      "Temp1", wrist1Temp.refresh().getValueAsDouble(), "Percent Output1", getWristMotorPercentOutput(),
      "Amps1", wrist1StatorCurrent.refresh().getValueAsDouble(),
      "Temp2", wrist2Temp.refresh().getValueAsDouble(), "Percent Output2", wrist2DutyCycle.refresh().getValueAsDouble(),
      "Amps2", wrist2StatorCurrent.refresh().getValueAsDouble(),
      "Volts1", wrist1MotorVotage.refresh().getValueAsDouble(),
      "Enc Pos Raw", getWristEncoderRotationsRaw(),
      "Enc Vel Raw", wrist1EncoderVelocity.refresh().getValueAsDouble(),
      "Enc Accel Raw", wrist1EncoderAcceleration.refresh().getValueAsDouble(),
      "WristCalZero", wristCalZero,
      "Wrist Degrees", getWristEncoderDegrees(),
      "Wrist2 Degrees", getWristEncoder2Degrees(),
      "Wrist Angle", getWristAngle(), "Wrist Target", getCurrentWristTarget(),
      "Rev Connected", isRevEncoderConnected(), "Rev Degrees", getRevEncoderDegrees(),
      "Lower Limit 1", isWristAtLowerLimit1(), "Lower Limit 2", isWristAtLowerLimit2()
    );
  }

  /**
   * Turns file logging on every scheduler cycle (~20ms) or every 10 cycles (~0.2 sec)
   * @param enabled true = every cycle, false = every 10 cycles
   */ 
  @Override
  public void enableFastLogging(boolean enabled) {
    fastLogging = enabled;
  }
  
  @Override
  public void periodic() {
    if (log.isMyLogRotation(logRotationKey)) {
      SmartDashboard.putBoolean("Wrist Rev connected", isRevEncoderConnected());
      SmartDashboard.putBoolean("Wrist calibrated", wristCalibrated);
      // SmartDashboard.putBoolean("Wrist LL1", isWristAtLowerLimit1());
      // SmartDashboard.putBoolean("Wrist LL2", isWristAtLowerLimit2());
      SmartDashboard.putBoolean("Wrist lower limit", isWristAtLowerLimit());
      SmartDashboard.putNumber("Wrist Rev angle", getRevEncoderDegrees());
      SmartDashboard.putNumber("Wrist Rev raw", revEncoder.get()*360.0);
      SmartDashboard.putNumber("Wrist angle", getWristEncoderDegrees());
      SmartDashboard.putNumber("Wrist target angle", getCurrentWristTarget());
      SmartDashboard.putNumber("Wrist enc1 raw", getWristEncoderRotationsRaw());
      SmartDashboard.putNumber("Wrist enc2 raw", wrist2EncoderPostion.refresh().getValueAsDouble());
      SmartDashboard.putNumber("Wrist output1", getWristMotorPercentOutput());
      SmartDashboard.putNumber("Wrist output2", wrist2DutyCycle.refresh().getValueAsDouble());
    }
        
    if (fastLogging || log.isMyLogRotation(logRotationKey)) {
      updateWristLog(false);
    }

    // Rev Through-Bore Encoder takes a while to boot up.
    // After it boots up, it takes up to 40ms sec to settle into an accurate reading.
    // Previously, we waited for 5 periodic cycles after the encoder boots up before calibrating.
    // Now, just wait 2.5 seconds after the Wrist constructor.  If the Rev encoder is still not
    // reading, then just use the hard stop angle.
    if (!wristCalibrated && bootTimer.hasElapsed(2.5) && isWristAtLowerLimit()) {
      if (isRevEncoderConnected()) {
        // Calibrate Rev encoder
        log.writeLogEcho(true, subsystemName, "calibrateEncoder pre", "Rev encoder connecected", true,
          "Pre Rev angle", getRevEncoderDegrees());

        calibrateRevEncoderDegrees(revEncoderOffsetAngleWrist);

        // Copy calibration to wrist encoder
        // This sets wristCalibrated to true
        calibrateWristEnc(getRevEncoderDegrees());

        log.writeLogEcho(true, subsystemName, "calibrateEncoder post", "Rev encoder connecected", true,
          "Post Rev angle", getRevEncoderDegrees(), "Post wrist angle", getWristAngle());
      } else {
        // Wrist is at lower limit, but Rev encoder is not working.  Assume wrist is on the hard stop.
        calibrateWristEnc(WristAngle.lowerLimit.value);

        log.writeLogEcho(true, subsystemName, "calibrateEncoder post", "Rev encoder connecected", false,
          "Post wrist angle", getWristAngle());
      }

      // Configure soft limits on motor
      wristMotor1Config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = wristDegreesToEncoderRotations(WristAngle.upperLimit.value);
      wristMotor1Config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = wristDegreesToEncoderRotations(WristAngle.lowerLimit.value);
      wristMotor1Config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
      wristMotor1Config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;

      // Apply configuration to the wrist motor. 1 and 2 
      // This is a blocking call and will wait up to 50ms-70ms for the config to apply.  (initial test = 62ms delay)
      wristMotor1Configurator.apply(wristMotor1Config);
    }

    // If driver station is no longer disabled and wrist is not calibrated, then 
    // record a sticky fault (once)
    if (!calibrationStickyFaultReported && !wristCalibrated && !DriverStation.isDisabled()) {
      calibrationStickyFaultReported = true;
      RobotPreferences.recordStickyFaults("Wrist-Not-calibrated-when-enabled", log);
      log.writeLogEcho(true, subsystemName, "calibrate Wrist", "Wrist calibrated", false);
    }

    // If the driver station is disabled, then turn off any position control for the wrist motor
    if (DriverStation.isDisabled()) {
      stopWrist();
    }

    // If the wrist hits the bump switch, then stop the wrist from moving down further
    if (isWristAtLowerLimit()) {
      if (wristCalibrated && isWristMotorPositionControl()) {
        // Wrist is calibrated and under position control, so don't let the position go down any further
        if (getCurrentWristTarget() < getWristEncoderDegrees()) {
          // Current target angle is below current angle.  Reset target to current angle
          setWristAngle(getWristEncoderDegrees());
        }
      } else {
        // Wrist is under voltage (direct speed) control.  Stop it if it is moving down
        if (getWristMotorPercentOutput()<0) {
          stopWrist();
        }
      }
    }

    // Un-calibrates the wrist if the angle is outside of bounds.
    // Turned off for right now.  It still occasionally false-triggers, even with 20degree tolerances.
    // if (getWristAngle() > WristAngle.upperLimit.value + 20.0 || getWristAngle() < WristAngle.lowerLimit.value - 20.0) {
    //   setWristUncalibrated();
    //   updateWristLog(true);
    // }

   
    
  }
 
}