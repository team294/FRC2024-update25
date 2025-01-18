// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.HashMap;
import java.util.Map;

import com.ctre.phoenix.led.Animation;
import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.RainbowAnimation;

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
  private Timer matchTimer;
  private CANdleEvents previousEventCANdle;
  private StripEvents previousEventStrip;
  private boolean lastStickyFaultPresentReading = false;

  public enum CANdleEvents {
    STICKY_FAULTS_CLEARED,
    STICKY_FAULT_PRESENT,
    WRIST_UNCALIBRATED,
    WRIST_CALIBRATED
  }

  Map<CANdleEvents, Integer> prioritiesCANdleEvents = Map.of(
    CANdleEvents.STICKY_FAULTS_CLEARED, 1,
    CANdleEvents.STICKY_FAULT_PRESENT, 1,
    CANdleEvents.WRIST_UNCALIBRATED, 1,
    CANdleEvents.WRIST_CALIBRATED, 1
);

  public enum StripEvents {
    INTAKING,
    PIECE_PRESENT,
    SHOOTER_WITHIN_TARGET_VELOCITY,
    RAINBOW,
    MATCH_COUNTDOWN,
    IDLE,
    ROBOT_DISABLED
  }

  Map<StripEvents, Integer> prioritiesStripEvents = Map.of(
    StripEvents.ROBOT_DISABLED, 0,
    StripEvents.INTAKING, 1,
    StripEvents.PIECE_PRESENT, 2,
    StripEvents.SHOOTER_WITHIN_TARGET_VELOCITY, 3,
    StripEvents.MATCH_COUNTDOWN, 4,
    StripEvents.RAINBOW, 5,
    StripEvents.IDLE, 6
  );

  /**
   * Get the priority level for an event.
   * @param event CANdleEvents event
   * @return priority level integer (higher value = higher priority), default is -1
   */
  private int getPriority(CANdleEvents event) {
    return event != null ? prioritiesCANdleEvents.getOrDefault(event, -1) : -1;
  }

  /**
   * Get the priority level for an event.
   * @param event StripEvents event
   * @return priority level integer (higher value = higher priority), default is -1
   */
  private int getPriority(StripEvents event) {
    return event != null ? prioritiesStripEvents.getOrDefault(event, -1) : -1;
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
    this.matchTimer = matchTimer;
    this.log = log;
    logRotationKey = log.allocateLogRotation();

    // Create the LED segments
    for (LEDSegmentRange segment : LEDSegmentRange.values()) {
      segments.put(segment, new LEDSegment(segment.index, segment.count, LEDConstants.Patterns.noPatternAnimation));
    }

    sendEvent(StripEvents.IDLE);
  }

  /**
   * Update strips for a countdown animation.
   * @param percent 0-1 progress through the countdown
   */
  private void updateLEDsCountdown(double percent) {

    double leftCount = LEDSegmentRange.StripLeft.count * percent;
    int ledCountLeft = (int) leftCount;

    double rightCount = LEDSegmentRange.StripRight.count * percent;
    int ledCountRight = (int) rightCount;

    setLEDs(Color.kRed, LEDSegmentRange.StripLeft.index + LEDSegmentRange.StripLeft.count - ledCountLeft, ledCountLeft);
    setLEDs(Color.kRed, LEDSegmentRange.StripRight.index, ledCountRight);
  }

  /**
   * Change the color of the LEDs.
   * @param color BCRColor to make LEDs (solid)
   * @param strip true = update strips, false = update CANdle
   */
  private void updateLEDs(BCRColor color, boolean strip) {
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
   * Send an event to the CANdle and update LEDs if necessary.
   * @param event CANdle event
   */
  public void sendEvent(CANdleEvents event) {
    // Do not update if the new event is sticky fault present and the previous event caused a sticky fault
    if (previousEventCANdle == CANdleEvents.WRIST_UNCALIBRATED && event == CANdleEvents.STICKY_FAULT_PRESENT) return;

    // Do not update if the new event priority is less than the previous
    if (getPriority(event) < getPriority(previousEventCANdle)) return;

    switch (event) {
      case WRIST_UNCALIBRATED:
        updateLEDs(BCRColor.WRIST_UNCALIBRATED, false);
        break;
      case STICKY_FAULT_PRESENT:
        updateLEDs(BCRColor.STICKY_FAULT_PRESENT, false);
        break;
      default:
        updateLEDs(BCRColor.CANDLE_IDLE, false);
        break;
    }

    // Update previous event since we updated the LEDs
    previousEventCANdle = event;
  }

  /**
   * Send an event to the Strips and update LEDs if necessary.
   * @param event Strip event
   */
  public void sendEvent(StripEvents event) {
    // Always update state if previous event was idle
    // Do not update if last event was not idle and the new event priority is less than the previous
    if (previousEventStrip != StripEvents.IDLE && getPriority(event) < getPriority(previousEventStrip)) return;

    // Do not update if last event was piece present and new event is idle
    // This prevents LED flashing between orange and white when there is a piece
    if (previousEventStrip == StripEvents.PIECE_PRESENT && event == StripEvents.IDLE) return;

    if (previousEventStrip == StripEvents.ROBOT_DISABLED && (event == StripEvents.MATCH_COUNTDOWN || event == StripEvents.RAINBOW)) return;

    switch (event) {
      case MATCH_COUNTDOWN:
        // Percent of the way through the last 10 seconds of the match (125 seconds in)
        double percent = Math.max(matchTimer.get() - 125, 0) / 10.0;
        updateLEDsCountdown(percent);
        break;
      case RAINBOW:
        RainbowAnimation anim = new RainbowAnimation(1, .7, LEDSegmentRange.StripHorizontal.count, false, LEDSegmentRange.StripHorizontal.index);
        animate(anim);
        break;
      case SHOOTER_WITHIN_TARGET_VELOCITY:
        updateLEDs(BCRColor.SHOOTER_WITHIN_TARGET_VELOCITY, true);
        break;
      case PIECE_PRESENT:
        updateLEDs(BCRColor.PIECE_PRESENT, true);
        break;
      case INTAKING:
        updateLEDs(BCRColor.INTAKING, true);
        break;
      default:
        clearAnimation();
        updateLEDs(BCRColor.IDLE, true);
        break;
    }

    // Update previous event since we updated the LEDs
    previousEventStrip = event;
  }

  /** 
   * Get the subsystem's name.
   * @return subsystem name as a string
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

  public void setLEDs(Color color, int index, int count) {
    candle.setLEDs((int) (color.red * 255), (int) (color.green * 255), (int) (color.blue) * 255, 0, index, count);
  }

  /**
   * Sets LEDs using color and index value
   * @param color color to set
   * @param index index to start at
   */
  public void setLEDs(Color color, int index) {
    candle.setLEDs((int) (color.red * 255), (int) (color.green * 255), (int) (color.blue * 255), 0, index, 1);
  }

  /**
   * Sets LEDs using BCRColor and segment values
   * @param color BCRColor value
   * @param segment segment to light up (range)
   */
  public void setLEDs(BCRColor color, LEDSegmentRange segment) {
    candle.setLEDs(color.r, color.g, color.b, 0, segment.index, segment.count);
  }

  @Override
  public void periodic() {
    // only update every log rotation as opposed to every 20ms
    if(log.isMyLogRotation(logRotationKey)) { 
      // if there is a sticky fault, send sticky fault present event
      if (RobotPreferences.isStickyFaultActive()) {
        sendEvent(CANdleEvents.STICKY_FAULT_PRESENT);
        lastStickyFaultPresentReading = true;
      }
      // if there is not a sticky fault and there previously was, send sticky fault cleared event
      else if (!RobotPreferences.isStickyFaultActive() && lastStickyFaultPresentReading) {
        sendEvent(CANdleEvents.STICKY_FAULTS_CLEARED);
        lastStickyFaultPresentReading = false;
      }

      // if in last 10 seconds of match, send match countdown event
      if (matchTimer.get() > 125 && matchTimer.get() <= 135) {
        sendEvent(StripEvents.MATCH_COUNTDOWN);
      }
    }
  }
}