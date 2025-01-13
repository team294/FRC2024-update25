// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.HashMap;

import com.ctre.phoenix.led.Animation;
import com.ctre.phoenix.led.CANdle;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
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
  // private boolean shouldClear;
  private CANdleEvents previousEventCANdle;
  private StripEvents previousEventStrip;
  private boolean lastIsDisabledReading = false;
  private boolean lastStickyFaultReading = false;

  public enum CANdleEvents {
    stickyFaultsCleared,
    stickyFaultPresent,
    wristUncalibrated,
    wristCalibrated
  }

  public enum StripEvents {
    piecePresent,
    shooterWithinTargetVelocity,
    rainbow,
    lastTenSeconds,
    idle
  }

  private CANdleEvents[] candleEventsArray = CANdleEvents.values();
  private StripEvents[] stripEventsArray = StripEvents.values();

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
    // this.shouldClear = false;
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
    // this.shouldClear = false;
    this.log = log;
    logRotationKey = log.allocateLogRotation();

    // this.accuracyDisplayThreshold = 35;
    // this.accuracy = 0;

    // Create the LED segments
    for (LEDSegmentRange segment : LEDSegmentRange.values()) {
      segments.put(segment, new LEDSegment(segment.index, segment.count, LEDConstants.Patterns.noPatternAnimation));
    }
  }

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

  public void updateState(CANdleEvents event) {
    // initialize priorities to -1 in case of a null event
    int newEventPriority = -1;
    int previousEventPriority = -1;

    // check priority of previous and new events
    for (int i = 0; i < candleEventsArray.length; i++) {
      if (event == candleEventsArray[i]) {
        newEventPriority = i;
      }

      if (previousEventCANdle == candleEventsArray[i]) {
        previousEventPriority = i;
      }
    }

    // if new event priority is less than previous event priority, do not update
    if (newEventPriority < previousEventPriority) return;

    switch (event) {
      case wristCalibrated:
        updateLEDs(BCRColor.stickyFaultPresent, false);
        break;
      case wristUncalibrated:
        updateLEDs(BCRColor.wristUncalibrated, false);
        break;
      case stickyFaultPresent:
        updateLEDs(BCRColor.stickyFaultPresent, false);
        break;
      default:
        updateLEDs(BCRColor.CANdleDefault, false);
        break;
    }

    previousEventCANdle = event;
  }

  public void updateState(StripEvents event) {
    // Store the state
    currentState = robotState.getState();

    int newEventPriority = -1;
    int previousEventPriority = -1;

    for (int i = 0; i < stripEventsArray.length; i++) {
      if (event == stripEventsArray[i]) {
        newEventPriority = i;
      }

      if (previousEventStrip == stripEventsArray[i]) {
        previousEventPriority = i;
      }
    }

    // if new event priority is less than previous event priority, do not update
    if (newEventPriority < previousEventPriority) return;

    switch (event) {
      case lastTenSeconds:
        // Percent of the way through the last 10 seconds of the match (125 seconds in)
        Double percent = Math.max(matchTimer.get() - 125, 0) / 10.0;
        updateLastTenSecondsLEDs(percent);
        break;
      case rainbow:
        break;
      case shooterWithinTargetVelocity:
        updateLEDs(BCRColor.shooterWithinTargetVelocity, true);
        break;
      case piecePresent:
        updateLEDs(BCRColor.piecePresent, true);
        break;
      default: // idle
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

    previousEventStrip = event;
  }

  /** Get the subsystem's name
   * @return the name of the subsystem
   */
  public String getName() {
    return subsystemName;
  }
  
  /** Clear all LEDs */
  public void clearLEDs() {
    setLEDs(0, 0, 0);
  }

  /**
   * Clear the LEDs of a specific segment range
   * @param segment the segment to clear
   */
  public void clearLEDs(LEDSegmentRange segment) {
    setLEDs(0, 0, 0, segment);
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
   * @param anim
   */
  public void animate(Animation anim) {
    candle.animate(anim);
  }

  /**
   * Sets the pattern and resizes it to fit the LED strip
   * @param color the color to use
   * @param segment the segment to use
   */
  public void setColor(Color color, LEDSegmentRange segment) {
    Color[] pattern = {color};
    setPattern(pattern, segment);
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
   * Sets the animation for a given led segment
   * @param color color to display
   * @param segment segment to play animation on
   */
  public void setAnimation(Color color, LEDSegmentRange segment) {
    segments.get(segment).setAnimation(color);
  }
  
  /**
   * Sets the animation for a given led segment using BCRColor
   * @param color BCRColor to display
   * @param segment segment to play animation on
   */
  public void setAnimation(BCRColor color, LEDSegmentRange segment) {
    Color _color = new Color(color.r, color.g, color.b);
    segments.get(segment).setAnimation(_color);
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
   * Takes in color and sets correct RGB values
   * @param color color to set
   */
  public void setLEDs(Color color) {
    setLEDs((int) (color.red * 255), (int) (color.green * 255), (int) (color.blue * 255));
  }

  /**
   * Sets LEDs using RGB, index, and count values
   * @param r red value
   * @param g green value
   * @param b blue value
   * @param index index to start at
   * @param count number of LEDs
   */
  public void setLEDs(int r, int g, int b, int index, int count) {
    candle.setLEDs(r, g, b, 0, index, count);
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
   * Sets LEDs using color, index, and count values
   * @param color color to set
   * @param index index to start at
   * @param count number of LEDs
   */
  public void setLEDs(Color color, int index, int count) {
    candle.setLEDs((int) (color.red * 255), (int) (color.green * 255), (int) (color.blue * 255), 0, index, count);
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
   * Sets LEDs using BCR color enum (ex: IDLE)
   * @param color color to set
   */
  public void setLEDs(BCRColor color) {
    candle.setLEDs(color.r, color.g, color.b);
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
    if(log.isMyLogRotation(logRotationKey)) {
      if (RobotPreferences.isStickyFaultActive()) {
        updateState(CANdleEvents.stickyFaultPresent);
        lastStickyFaultReading = true;
      }
      else if (!RobotPreferences.isStickyFaultActive() && lastStickyFaultReading) {
        updateState(CANdleEvents.stickyFaultsCleared);
        lastStickyFaultReading = false;
      }

      // non-permanent piece detection when robot is disabled
      if (DriverStation.isDisabled() && !lastIsDisabledReading) {
        updateState(StripEvents.idle);
        lastIsDisabledReading = true;
      }
      else if (!DriverStation.isDisabled() && lastIsDisabledReading) {
        lastIsDisabledReading = false;
      }
    }
  }
}