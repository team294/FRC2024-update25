/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.VoltageOut;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.Ports;
import frc.robot.subsystems.LED.StripEvents;
import frc.robot.utilities.FileLog;
import frc.robot.utilities.Loggable;
import static frc.robot.utilities.StringUtil.*;


public class Intake extends SubsystemBase implements Loggable {

  private final FileLog log;
  private final int logRotationKey;
  private Timer currentTimer = new Timer();
  private boolean fastLogging = false; // true is enabled to run every cycle; false follows normal logging cycles
  private String subsystemName;    // subsystem name for use in file logging and Shuffleboard

  // Create Falcon for centering motor
  private final TalonFX centeringMotor = new TalonFX(Constants.Ports.CANCenteringMotor);
  private final TalonFXConfigurator centeringConfigurator = centeringMotor.getConfigurator();
	private TalonFXConfiguration centeringConfig;
	private VoltageOut centeringVoltageControl = new VoltageOut(0.0);

  // Create Falcon variables for centering motor
	private final StatusSignal<Temperature> centeringTemp;				// Motor temperature, in degC
	private final StatusSignal<Double> centeringDutyCycle;				// Motor duty cycle percent power, -1 to 1
	private final StatusSignal<Current> centeringStatorCurrent;		// Motor stator current, in amps (+=fwd, -=rev)
	private final StatusSignal<Angle> centeringEncoderPosition;			// Encoder position, in pinion rotations
	private final StatusSignal<AngularVelocity> centeringEncoderVelocity;	
	private final StatusSignal<Voltage> centeringVoltage;

  // Create Falcon variables for intake motor
  private final TalonFX intakeMotor = new TalonFX(Constants.Ports.CANIntake);
  private final TalonFXConfigurator intakeConfigurator = intakeMotor.getConfigurator();
	private TalonFXConfiguration intakeConfig;
	private VoltageOut intakeVoltageControl = new VoltageOut(0.0);
  private final LED led;

  // Create Falcon variables for intake motor
  private final StatusSignal<Voltage> intakeSupplyVoltage;				// Incoming bus voltage to motor controller, in volts
	private final StatusSignal<Temperature> intakeTemp;				// Motor temperature, in degC
	private final StatusSignal<Double> intakeDutyCycle;				// Motor duty cycle percent power, -1 to 1
	private final StatusSignal<Current> intakeStatorCurrent;		// Motor stator current, in amps (+=fwd, -=rev)
	private final StatusSignal<Angle> intakeEncoderPosition;			// Encoder position, in pinion rotations
	private final StatusSignal<AngularVelocity> intakeEncoderVelocity;	
	private final StatusSignal<Voltage> intakeVoltage;

  // Piece sensor inside the intake
  private final DigitalInput pieceSensor = new DigitalInput(Ports.DIOIntakePieceSensor);
  
  /**
   * Creates the intake subsystem
   * @param subsystemName
   * @param log
   */
  public Intake(String subsystemName, LED led, FileLog log) {
    this.log = log; // save reference to the fileLog
    this.subsystemName = subsystemName;
    this.led = led;
    logRotationKey = log.allocateLogRotation();
    currentTimer.reset();
    currentTimer.start();

    // Get signal and sensor objects
    intakeSupplyVoltage = intakeMotor.getSupplyVoltage();
	  intakeTemp = intakeMotor.getDeviceTemp();
	  intakeDutyCycle = intakeMotor.getDutyCycle();
	  intakeStatorCurrent = intakeMotor.getStatorCurrent();
	  intakeEncoderPosition = intakeMotor.getPosition();
	  intakeEncoderVelocity = intakeMotor.getVelocity();
    intakeVoltage = intakeMotor.getMotorVoltage();

	  centeringTemp = centeringMotor.getDeviceTemp();
	  centeringDutyCycle = centeringMotor.getDutyCycle();
	  centeringStatorCurrent = centeringMotor.getStatorCurrent();
	  centeringEncoderPosition = centeringMotor.getPosition();
	  centeringEncoderVelocity = centeringMotor.getVelocity();
    centeringVoltage = centeringMotor.getMotorVoltage();
    
    // Configure the intake motor
    intakeConfig = new TalonFXConfiguration();			// Factory default configuration
    intakeConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;		// B1:  Intake re-design between AZ East and Aerospace valley:  invert motor
		intakeConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;          // Hold piece if we stop the motor
    intakeConfig.Voltage.PeakForwardVoltage = IntakeConstants.compensationVoltage;
    intakeConfig.Voltage.PeakReverseVoltage = -IntakeConstants.compensationVoltage;
    intakeConfig.OpenLoopRamps.VoltageOpenLoopRampPeriod = 0.3;         // # seconds from 0 to full power

    // Supply current limit is typically used to prevent breakers from tripping.
    intakeConfig.CurrentLimits.SupplyCurrentLimit = 60.0;       // (amps) If current is above this value for longer than threshold time, then limit current to the lower limit
    intakeConfig.CurrentLimits.SupplyCurrentLowerLimit = 35.0;   // (amps) Lower limit for the current
    intakeConfig.CurrentLimits.SupplyCurrentLowerTime = 0.2;       // (sec) Threshold time
    intakeConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

    // Set intake motor sensor
    intakeConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;  // use built-in encoder

    // set intake motor configuration
    intakeConfigurator.apply(intakeConfig);


    // Configure the centering motor
    centeringConfig = new TalonFXConfiguration();			// Factory default configuration
    centeringConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;		// Don't invert motor
		centeringConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;          // Hold piece if we stop the motor
    centeringConfig.Voltage.PeakForwardVoltage = IntakeConstants.compensationVoltage;
    centeringConfig.Voltage.PeakReverseVoltage = -IntakeConstants.compensationVoltage;
    centeringConfig.OpenLoopRamps.VoltageOpenLoopRampPeriod = 0.3;         // # seconds from 0 to full power

    // Set centering motor sensor
    centeringConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;  // use built-in encoder

    // set centering motor configuration
    centeringConfigurator.apply(centeringConfig);


    stopIntakeMotor();
    stopCenteringMotor();
  }

  /**
   * Returns the name of the subsystem
   */
  public String getName() {
    return subsystemName;
  }

  /**
   * sets the percent of the intake motor, using voltage compensation
   * @param percent percent -1.0 to 1.0 (+ = intake, - = outtake)
   */
  public void setIntakePercentOutput(double percent){
    intakeMotor.setControl(intakeVoltageControl.withOutput(percent*IntakeConstants.compensationVoltage));
    if (percent > 0) {
      led.sendEvent(StripEvents.INTAKING);
    }
    else {
      led.sendEvent(StripEvents.IDLE);
    }
  }

  /**
   * Sets the percent output of the centering motor
   * @param percent percent -1.0 to 1.0 (+ = intake, - = outtake)
   */
  public void setCenteringMotorPercentOutput(double percent){
    centeringMotor.setControl(centeringVoltageControl.withOutput(percent*IntakeConstants.compensationVoltage));
  }

  /**
  * Stops the intake motor
  */
  public void stopIntakeMotor(){
    setIntakePercentOutput(0);
  }

  /**
   * Stops the centering motor
   */
  public void stopCenteringMotor(){
    setCenteringMotorPercentOutput(0);
  }

  /**
   * Returns the intake motor position
   * @return position of motor in revolutions
   */
  public double getIntakePosition(){
    intakeEncoderPosition.refresh();
    return intakeEncoderPosition.getValueAsDouble();
  }

  /**
   * Returns the centering motor position
   * @return position of the motor in revolutions
   */
  public double getCenteringMotorPosition(){
    centeringEncoderPosition.refresh();
    return centeringEncoderPosition.getValueAsDouble();
  }
    
  /**
   * Get the velocity of the intake motor
   * @return velocity of motor in RPM
   */
  public double getIntakeVelocity(){
    intakeEncoderVelocity.refresh();
    return intakeEncoderVelocity.getValueAsDouble() * 60.0;
  }

  /**
   * Get the velocity of the centering motor
   * @return velocity of centering motor in RPM
   */
  public double getCenteringMotorVelocity(){
    centeringEncoderVelocity.refresh();
    return centeringEncoderVelocity.getValueAsDouble() * 60.0;
  }

  /**
   * Gets the intake motor current
   * @return Intake motor current, in amps
   */
  public double getIntakeAmps() {
    intakeStatorCurrent.refresh();
    return intakeStatorCurrent.getValueAsDouble();
  }

  /**
   * 
   * @return true if piece is in intake
   */
  public boolean isPiecePresent(){
    return !pieceSensor.get();
  }

  @Override
  public void periodic(){
    if(fastLogging || log.isMyLogRotation(logRotationKey)) {
      updateLog(false);
    }

    if(log.isMyLogRotation(logRotationKey)) {
        SmartDashboard.putNumber(buildString(subsystemName, " Voltage"), intakeVoltage.refresh().getValueAsDouble());
        // SmartDashboard.putNumber(buildString(subsystemName, " Position Rev"), getIntakePosition());
        SmartDashboard.putNumber(buildString(subsystemName, " Velocity RPM"), getIntakeVelocity());
        SmartDashboard.putNumber(buildString(subsystemName, " Temperature C"), intakeTemp.refresh().getValueAsDouble());
        SmartDashboard.putNumber("Centering Voltage", centeringVoltage.refresh().getValueAsDouble());
        // SmartDashboard.putNumber("Centering Position Rev", getCenteringMotorPosition());
        SmartDashboard.putNumber("Centering Velocity RPM", getCenteringMotorVelocity());
        SmartDashboard.putNumber("Centering Temperature C", centeringTemp.refresh().getValueAsDouble());
        SmartDashboard.putBoolean(buildString(subsystemName, " Is Piece Present"), isPiecePresent());
    }

    // Reset safety time if current is below the threshhold
    if(Math.abs(getIntakeAmps()) < IntakeConstants.intakingPieceCurrentThreshold) {
      currentTimer.reset();
    } else {
      log.writeLog(false, subsystemName, "Intake current above limit", "Amps", getIntakeAmps());
    }

    // If current is above threshold for too long, then shut off intake motor to prevent overheating.
    // This will also also the centering and feeder motors to pull the piece in after a jam.
    if(currentTimer.hasElapsed(0.5)) {
      this.stopIntakeMotor();
      currentTimer.reset();
      log.writeLog(false, subsystemName, "Intake shutoff");
    }
  }

  @Override
  public void enableFastLogging(boolean enabled) {
    fastLogging = enabled;
  }

  /**
   * Write information about intake to fileLog.
   * @param logWhenDisabled true = log when disabled, false = discard the string
   */
	public void updateLog(boolean logWhenDisabled) {
        log.writeLog(logWhenDisabled, subsystemName, "Update Variables",  
      "Bus Volt", intakeSupplyVoltage.refresh().getValueAsDouble(),
      "Intake Percent", intakeDutyCycle.refresh().getValueAsDouble(),
      "Centering Percent", centeringDutyCycle.refresh().getValueAsDouble(),
      "Intake Amps", getIntakeAmps(),
      "Centering Amps", centeringStatorCurrent.refresh().getValueAsDouble(),
      "Intake Temperature", intakeTemp.refresh().getValueAsDouble(),
      "Centering Temperature", centeringTemp.refresh().getValueAsDouble(),
      "Intake Position", getIntakePosition(),
      "Centering Position", getCenteringMotorPosition(),
      "Intake RPM", getIntakeVelocity(),
      "Centering RPM", getCenteringMotorVelocity()
    );
  }

  
}