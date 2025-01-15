// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.HashMap;
import java.util.Map;

import com.ctre.phoenix.led.Animation;
import com.ctre.phoenix.led.CANdle;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.BCRColor;
import frc.robot.Constants.LEDConstants;
import frc.robot.Constants.LEDConstants.*;
import frc.robot.utilities.BCRRobotState;
import frc.robot.utilities.FileLog;
import frc.robot.utilities.LEDSegment;
import frc.robot.utilities.RobotPreferences;


public class LED extends SubsystemBase {
  private final FileLog log;
  private final int logRotationKey;
  private final CANdle candle;
  private String subsystemName;
  private BCRRobotState robotState;
  private BCRRobotState.State currentState;
  private Timer matchTimer;
  private CANdleEvents previousEventCANdle;
  private StripEvents previousEventStrip;
  private boolean lastIsDisabledReading = false;
  private boolean lastStickyFaultPresentReading = false;

  public enum CANdleEvents {
    STICKY_FAULTS_CLEARED,
    STICKY_FAULT_PRESENT,
    WRIST_UNCALIBRATED,
    WRIST_CALIBRATED
  }

  Map<CANdleEvents, Integer> candleEventPriorities = Map.of(
    CANdleEvents.STICKY_FAULTS_CLEARED, 1,
    CANdleEvents.STICKY_FAULT_PRESENT, 1,
    CANdleEvents.WRIST_UNCALIBRATED, 2,
    CANdleEvents.WRIST_CALIBRATED, 2
);

  public enum StripEvents {
    PIECE_PRESENT,
    SHOOTER_WITHIN_TARGET_VELOCITY,
    RAINBOW,
    LAST_TEN_SECONDS,
    IDLE
  }

  Map<StripEvents, Integer> stripEventPriorities = Map.of(
    StripEvents.PIECE_PRESENT, 1,
    StripEvents.SHOOTER_WITHIN_TARGET_VELOCITY, 2,
    StripEvents.RAINBOW, 3,
    StripEvents.LAST_TEN_SECONDS, 4,
    StripEvents.IDLE, 5
  );

  private int getPriority(CANdleEvents event) {
    return candleEventPriorities.getOrDefault(event, -1);
  }

  private int getPriority(StripEvents event) {
    return stripEventPriorities.getOrDefault(event, -1);
  }

  // private Color[] accuracyDisplayPattern = {Color.kRed, Color.kRed};
  private HashMap<LEDSegmentRange, LEDSegment> segments;

  /**
   * Creates the CANdle LED subsystem.
   * @param CANPort
   * @param subsystemName
   * @param matchTimer
   * @param log
   */
  public LED(int CANPort, String subsystemName, Timer matchTimer, FileLog log, BCRRobotState robotState) {
    this.subsystemName = subsystemName;
    this.candle = new CANdle(CANPort, "");
    this.segments = new HashMap<LEDSegmentRange, LEDSegment>();
    this.robotState = robotState;
    this.currentState = BCRRobotState.State.IDLE;
    this.matchTimer = matchTimer;
    this.log = log;
    logRotationKey = log.allocateLogRotation();

    // this.accuracyDisplayThreshold = 35;
    // this.accuracy = 0;

    // Create the LED segments
    for (LEDSegmentRange segment : LEDSegmentRange.values()) {
      segments.put(segment, new LEDSegment(segment.index, segment.count, LEDConstants.Patterns.noPatternAnimation));
    }
  }

  /**
   * Creates the CANdle LED subsystem without requiring a Timer object.
   * @param CANPort
   * @param subsystemName
   * @param log
   */
  public LED(int CANPort, String subsystemName, FileLog log, BCRRobotState robotState) {
    this.subsystemName = subsystemName;
    this.candle = new CANdle(CANPort, "");
    this.segments = new HashMap<LEDSegmentRange, LEDSegment>();
    this.robotState = robotState;
    this.currentState = BCRRobotState.State.IDLE;
    this.log = log;
    logRotationKey = log.allocateLogRotation();

    // this.accuracyDisplayThreshold = 35;
    // this.accuracy = 0;

    // Create the LED segments
    for (LEDSegmentRange segment : LEDSegmentRange.values()) {
      segments.put(segment, new LEDSegment(segment.index, segment.count, LEDConstants.Patterns.noPatternAnimation));
    }
  }

  /**
   * Update strips in the last 10 seconds of the match
   * @param percent percent of the way through the last 10 seconds
   */
  public void updateLastTenSecondsLEDs(double percent) {
    // Generates segment pattern for the left vertical segment based on percent
    Color[] segmentPatternLeft = new Color[LEDSegmentRange.StripLeft.count];
    for (int i = 0; i < LEDSegmentRange.StripLeft.count; i++) {
      if (i >= (1.0 - percent) * LEDSegmentRange.StripLeft.count) {
        segmentPatternLeft[i] = Color.kRed;
      } else {
        Color[] frame = segments.get(LEDSegmentRange.StripLeft).getCurrentFrame();
        segmentPatternLeft[i] = frame[Math.max(Math.min(frame.length - 1, i), 0)];
      }
    }
    // Generates segment pattern for the right vertical segment based on percent
    Color[] segmentPatternRight = new Color[LEDSegmentRange.StripRight.count];
    for (int i = 0; i < LEDSegmentRange.StripRight.count; i++) {
      if (i < percent * LEDSegmentRange.StripRight.count) {
        segmentPatternRight[i] = Color.kRed;
      } else {
        Color[] frame = segments.get(LEDSegmentRange.StripRight).getCurrentFrame();
        segmentPatternRight[i] = frame[Math.max(Math.min(frame.length - 1, i), 0)];
      }
    }
    // Generates segment pattern for the horizontal segment based on percent
    Color[] segmentPatternHorizontal = new Color[LEDSegmentRange.StripHorizontal.count];
    for (int i = 0; i < LEDSegmentRange.StripHorizontal.count; i++) {
      if (i < percent * LEDSegmentRange.StripHorizontal.count) {
        segmentPatternHorizontal[i] = Color.kRed;
      } else {
        Color[] frame = segments.get(LEDSegmentRange.StripHorizontal).getCurrentFrame();
        segmentPatternHorizontal[i] = frame[Math.max(Math.min(frame.length - 1, i), 0)];
      }
    }
    // Sets segments based on generated patterns
    setAnimation(segmentPatternLeft, LEDSegmentRange.StripLeft, true);
    setAnimation(segmentPatternRight, LEDSegmentRange.StripRight, true);
    setAnimation(segmentPatternHorizontal, LEDSegmentRange.StripHorizontal, true);
  }

  /**
   * Change the color of the LEDs based on parameters
   * @param color BCRColor to use (see enum in constants)
   * @param strip true = update strips, false = update CANdle
   */
  public void updateLEDs(BCRColor color, boolean strip) {
    if (strip) {
      setLEDs(color, LEDSegmentRange.StripLeft);
      setLEDs(color, LEDSegmentRange.StripRight);
      setLEDs(color, LEDSegmentRange.StripHorizontal);
    }
    else {
      setLEDs(color, LEDSegmentRange.CANdle);
    }
  }

  /**
   * Update the CANdle based on the state
   * @param event CANdle event to use
   */
  public void updateState(CANdleEvents event) {
    // if new event priority is less than previous, do not update
    if (getPriority(event) < getPriority(previousEventCANdle)) return;

    switch (event) {
      case WRIST_UNCALIBRATED:
        updateLEDs(BCRColor.WRIST_UNCALIBRATED, false);
        break;
      case STICKY_FAULT_PRESENT:
        updateLEDs(BCRColor.STICKY_FAULT_PRESENT, false);
        break;
      default:
        updateLEDs(BCRColor.CANDLE_DEFAULT, false);
        break;
    }

    // Update previous event for CANdle to event that just happened
    previousEventCANdle = event;
  }

  /**
   * Update the LED Strips based on the state
   * @param event Strip event to use
   */
  public void updateState(StripEvents event) {
    // Store the state
    currentState = robotState.getState();

    // Do not update state if last event was not idle and the new priority is 
    // less than the previous. Always update state if previous event was idle.
    if (previousEventStrip != StripEvents.IDLE && getPriority(event) < getPriority(previousEventStrip)) return;

    switch (event) {
      case LAST_TEN_SECONDS: // TODO this is never sent, fix
        // Percent of the way through the last 10 seconds of the match (125 seconds in)
        Double percent = Math.max(matchTimer.get() - 125, 0) / 10.0;
        updateLastTenSecondsLEDs(percent);
        break;
      case RAINBOW:
        break;
      case SHOOTER_WITHIN_TARGET_VELOCITY:
        updateLEDs(BCRColor.SHOOTER_WITHIN_TARGET_VELOCITY, true);
        break;
      case PIECE_PRESENT:
        updateLEDs(BCRColor.PIECE_PRESENT, true);
        break;
      default: // idle TODO doesn't work, solution theorized
        switch (currentState) {
          case SHOOTING:
            updateLEDs(BCRColor.SHOOTING, true);
            break;
          case INTAKING:
            updateLEDs(BCRColor.INTAKING, true);
            break;
          case IDLE:
            updateLEDs(BCRColor.IDLE, true);
            break;
        }
        break;
    }

    // Update previous event for strips to event that just happened
    previousEventStrip = event;
  }

  /** 
   * Get the subsystem's name
   * @return the name of the subsystem
   */
  public String getName() {
    return subsystemName;
  }
  
  /**
   * Clear all animation
   */
  public void clearAnimation() {
    candle.clearAnimation(0);
    for (LEDSegmentRange segmentKey : segments.keySet()) {
      setAnimation(LEDConstants.Patterns.noPatternAnimation, segmentKey, false);
    }
  }
  
  /**
   * Start a built-in animation
   * @param anim animation object to use
   */
  public void animate(Animation anim) {
    candle.animate(anim);
  }

  /**
   * Sets the pattern and resizes it to fit the LED strip
   * @param pattern the pattern to use
   * @param segment the segment to use
   */
  public void setPattern(Color[] pattern, LEDSegmentRange segment) {
    if (pattern.length == 0) return;
    for (int indexLED = 0, indexPattern = 0; indexLED < segment.count; indexLED++, indexPattern++) {
      if (indexPattern >= pattern.length) indexPattern = 0;
      setLEDs(pattern[indexPattern], segment.index + indexLED);
    }
  }

  /**
   * Sets the animation for a given led segment
   * @param animation animation to display
   * @param segment segment to play animation on
   * @param loop whether the animation repeats
   */
  public void setAnimation(Color[][] animation, LEDSegmentRange segment, boolean loop) {
    segments.get(segment).setAnimation(animation, loop);
  }

   /**
   * Sets the animation for a given LED segment using Color
   * @param pattern pattern to display
   * @param segment segment to play animation on
   * @param loop whether the animation repeats
   */
  public void setAnimation(Color[] pattern, LEDSegmentRange segment, boolean loop) {
    Color[][] anim = {pattern};
    segments.get(segment).setAnimation(anim, loop);
  }

  /**
   * Sets LEDs using only R, G, and B
   * @param r red value
   * @param g green value
   * @param b blue value
   */
  public void setLEDs(int r, int g, int b) {
    candle.setLEDs(r, g, b);
  }

  /**
   * Sets LEDs using color and index values
   * @param color color to set
   * @param index index to start at
   */
  public void setLEDs(Color color, int index) {
    candle.setLEDs((int) (color.red * 255), (int) (color.green * 255), (int) (color.blue * 255), 0, index, 1);
  }
  
  /**
   * Sets LEDs using RGB and segment values
   * @param r red value
   * @param g green value
   * @param b blue value
   * @param segment segment to light up (range)
   */
  public void setLEDs(int r, int g, int b, LEDSegmentRange segment) {
    candle.setLEDs(r, g, b, 0, segment.index, segment.count);
  }
  
  /**
   * Sets LEDs using robot state (ex: IDLE)
   * @param color color to set
   * @param index index to start at
   * @param count number of LEDs
   */
  public void setLEDs(BCRColor color, int index, int count) {
    candle.setLEDs(color.r, color.g, color.b, 0, index, count);
  }

  public void setLEDs(BCRColor color, LEDSegmentRange segment) {
    candle.setLEDs(color.r, color.g, color.b, 0, segment.index, segment.count);
  }

  @Override
  public void periodic() {
    // only update every log rotation as opposed to every 20ms
    if(log.isMyLogRotation(logRotationKey)) { 
      // if there is a sticky fault, send sticky fault event
      if (RobotPreferences.isStickyFaultActive()) { // TODO revisit this to see if it's necessary
        updateState(CANdleEvents.STICKY_FAULT_PRESENT);
        lastStickyFaultPresentReading = true;
      }
      // if there is no longer a sticky fault, send sticky fault cleared event
      // in other words, if sticky faults are cleared, send clear event
      else if (!RobotPreferences.isStickyFaultActive() && lastStickyFaultPresentReading) {
        updateState(CANdleEvents.STICKY_FAULTS_CLEARED);
        lastStickyFaultPresentReading = false;
      }
    }
  }
}