// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.*;
import frc.robot.subsystems.LED.StripEvents;
import frc.robot.utilities.FileLog;
import frc.robot.utilities.Loggable;
import frc.robot.utilities.StringUtil;

public class Feeder extends SubsystemBase implements Loggable{
  private final FileLog log;
  private boolean fastLogging = false;
  private int logRotationKey;
  private final String subsystemName;
  private final LED led;

  // Create Kraken for feeder motor
  private final TalonFX feeder = new TalonFX(Ports.CANFeeder);
  private final TalonFXConfigurator feederConfigurator;
  private TalonFXConfiguration feederConfig;

  // Create signals and sensors for the feeder motor
  private final StatusSignal<Voltage> feederSupplyVoltage;				// Incoming bus voltage to motor controller, in volts
	private final StatusSignal<Temperature> feederTemp;				// Motor temperature, in degC
	private final StatusSignal<Double> feederDutyCycle;				// Motor duty cycle percent power, -1 to 1
	private final StatusSignal<Current> feederStatorCurrent;		// Motor stator current, in amps (+=fwd, -=rev)
	private final StatusSignal<Angle> feederEncoderPosition;			// Encoder position, in pinion rotations
	private final StatusSignal<AngularVelocity> feederEncoderVelocity;
  private final StatusSignal<Voltage> feederVoltage;

  // Motor controls
  private VoltageOut motorVoltageControl = new VoltageOut(0.0);
  private VelocityVoltage motorVelocityControl = new VelocityVoltage(0.0).withSlot(0);

  private boolean velocityControlOn = false;
  private double setpointRPM;
  private double setpointPercent;
  private boolean lastPiecePresentReading; // true = piece was present in feeder

  // Piece sensor inside the intake 
  private final DigitalInput pieceSensor = new DigitalInput(Ports.DIOFeederPieceSensor);

  /** Creates a new Feeder. */
  public Feeder(FileLog log, LED led) {
    this.log = log;
    logRotationKey = log.allocateLogRotation();
    subsystemName = "Feeder";
    this.led = led;

    // Configure feeder
    feederConfigurator = feeder.getConfigurator();
    feederSupplyVoltage = feeder.getSupplyVoltage();
	  feederTemp = feeder.getDeviceTemp();
	  feederDutyCycle = feeder.getDutyCycle();
	  feederStatorCurrent =feeder.getStatorCurrent();
	  feederEncoderPosition = feeder.getPosition();
	  feederEncoderVelocity = feeder.getVelocity();
    feederVoltage = feeder.getMotorVoltage();

    feederConfig = new TalonFXConfiguration();			// Factory default configuration
    feederConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;		// Don't invert motor
		feederConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;      // Brake mode to hold piece in feeder
    feederConfig.OpenLoopRamps.VoltageOpenLoopRampPeriod = 0.1;         // # seconds from 0 to full power
		feederConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 0.1;     // # seconds from 0 to full power
    
    // Set PID Coefficients
    feederConfig.Slot0.kP = FeederConstants.kP;
    feederConfig.Slot0.kI = FeederConstants.kI;
    feederConfig.Slot0.kD = FeederConstants.kD;
    feederConfig.Slot0.kS = FeederConstants.kS;
    feederConfig.Slot0.kV = FeederConstants.kV;
    feederConfig.Slot0.kA = FeederConstants.kA;

    // Apply configuration to feeder motor.  
		// This is a blocking call and will wait up to 50ms-70ms for each config to apply.  (initial test = 62ms delay)
    feederConfigurator.apply(feederConfig);

    // Stop Feeder Motor
    stopFeeder();

    lastPiecePresentReading = false;
  }

  /**
   * Returns the name of the subsystem
   */
  public String getName() {
    return subsystemName;
  }

  // *** Percent motor controls

  /**
   * Sets the percent of the feeder motor, using voltage compensation.
   * @param percent percent for the feeder motor, -1.0 to +1.0 (+ = feed forward, - = backwards)
   */
  public void setFeederPercentOutput(double percent) {
    // Percent output control does not exist; multiply compensationVoltage by percent
    feeder.setControl(motorVoltageControl.withOutput(percent * FeederConstants.compensationVoltage));
    velocityControlOn = false;
    setpointPercent = percent;
    setpointRPM = 0.0;
  }

  /**
  * Stops feeder motor
  */
  public void stopFeeder() {
    setFeederPercentOutput(0.0);
  }

  /**
   * Returns the setpoint percent for the feeder motor.
   * If the feeder is under velocity control, then returns 0.
   * @return
   */
  public double getFeederSetPercent() {
    return setpointPercent;
  }

  // *** Velocity motor controls

  /**
   * Sets the target velocity for feeder wheels
   * @param rpm velocity of feeder wheels in rpm
   */
  public void setFeederVelocity(double rpm) { 
    velocityControlOn = true;
    setpointRPM = rpm;
    setpointPercent = 0.0;
    feeder.setControl(motorVelocityControl.withVelocity(rpm/60.0/FeederConstants.feederGearRatio));
  }

  /**
   * @return difference between measured RPM and set point RPM for feeder wheels
   */
  public double getFeederVelocityPIDError() {
    return getFeederVelocity() - setpointRPM;
  }

  // *** Motor sensors

  /**
   * Returns the feeder position
   * @return position of feeder in wheel rotations
   */
  public double getFeederPosition() {
    feederEncoderPosition.refresh();
    return feederEncoderPosition.getValueAsDouble() * FeederConstants.feederGearRatio;
  }

  /**
   * @return velocity of feeder in wheel rpm
   */
  public double getFeederVelocity() {
    feederEncoderVelocity.refresh();
    return feederEncoderVelocity.getValueAsDouble() * 60.0 * FeederConstants.feederGearRatio;
  }

  /**
   * Reads the feeder voltage applied to the motor
   * @return feeder voltage, in volts
   */
  public double getFeederVoltage() {
    return feederVoltage.refresh().getValueAsDouble();
  }

  // ***  Piece sensor

  /**
   * @return true if piece is in feeder
   */
  public boolean isPiecePresent(){
    return !pieceSensor.get();
  }


  @Override
  public void periodic() {
    // Log
    if (fastLogging || log.isMyLogRotation(logRotationKey)) {
      updateLog(false);

      SmartDashboard.putNumber(StringUtil.buildString(subsystemName, " Voltage"), getFeederVoltage());
      SmartDashboard.putNumber(StringUtil.buildString(subsystemName, " RPM"), getFeederVelocity());
      SmartDashboard.putNumber(StringUtil.buildString(subsystemName, " Temp C"), feederTemp.refresh().getValueAsDouble());
      SmartDashboard.putBoolean("Feeder has piece", isPiecePresent());

      // if we have a piece and previously didn't have a piece, update state
      if (isPiecePresent() && !lastPiecePresentReading) {
        led.updateState(StripEvents.PIECE_PRESENT);
        lastPiecePresentReading = true;
      }
      // if we don't have a piece and previously did have a piece, update state
      else if (!isPiecePresent() && lastPiecePresentReading) {
          led.updateState(StripEvents.IDLE);
          lastPiecePresentReading = false;
      }
    }
  }

  /**
   * Update information about feeder to fileLog
   * @param logWhenDisabled true = log when disabled, false = discard the string
   */
  public void updateLog(boolean logWhenDisabled) {
    log.writeLog(
      logWhenDisabled,
      subsystemName,
      "Update Variables",
      "Bus Volt", feederSupplyVoltage.refresh().getValueAsDouble(),
      "Out Percent", feederDutyCycle.refresh().getValueAsDouble(),
      "Volt", feederVoltage.refresh().getValueAsDouble(),
      "Amps", feederStatorCurrent.refresh().getValueAsDouble(),
      "Temp", feederTemp.refresh().getValueAsDouble(),
      "Meas RPM", getFeederVelocity(),
      "Velocity Control", velocityControlOn,
      "Set RPM", setpointRPM,
      "Piece in Feeder", isPiecePresent()
    );
  }

  @Override
  public void enableFastLogging(boolean enabled) {
    fastLogging = enabled;
  }
}