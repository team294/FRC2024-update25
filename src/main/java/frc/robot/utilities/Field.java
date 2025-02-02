// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utilities;

import java.util.ArrayList;
import java.util.HashMap;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants;

/** Add your docs here. */
public class Field {

    private HashMap<String, Pose2d> ReefScoringPositions;
    private AllianceSelection allianceSelection;
    private final FileLog log;

    /**
     * Create a field object that can provide various field locations.  All field
     * locations are Pose2d objects based on the current alliance that is selected.
     * Pose components include:
     * <p> Robot X location in the field, in meters (0 = field edge in front of driver station, + = away from our drivestation)
     * <p> Robot Y location in the field, in meters (0 = right edge of field when standing in driver station, + = left when looking from our drivestation)
     * <p> Robot angle on the field (0 = facing away from our drivestation, + to the left, - to the right)
     * @param alliance Alliance object to provide the currently selected alliance
     */
    public Field(AllianceSelection allianceSelection, FileLog log){
        this.ReefScoringPositions = new HashMap<String, Pose2d>(12);
        this.allianceSelection = allianceSelection;
        this.log = log;

        //Reef Scoring Positions, points are on the walls of the reef (Positions are relative, with the origin being the corner to the driver's right)
        //0 is the closest left scoring position to the drivers, following positions moving clockwise
        //Angles represent the direction the robot would be facing (if scoring mechanism is on the front) when scoring at that point
        //
        //Drivers | Field     K  J
        //        |         L      I
        //        |       A          H
        //        |       B          G
        //        |         C      F
        //        |           D  E
        //(Rotation of 0 faces away from drivers)
        ReefScoringPositions.put("A", new Pose2d(Units.inchesToMeters(144.0),         Units.inchesToMeters(166.006493),     new Rotation2d(0)        ));
        ReefScoringPositions.put("B", new Pose2d(Units.inchesToMeters(144.0),         Units.inchesToMeters(153.292837),     new Rotation2d(0)        ));
        ReefScoringPositions.put("C", new Pose2d(Units.inchesToMeters(154.867597965), Units.inchesToMeters(134.469605169),  new Rotation2d(Math.PI/3)      ));
        ReefScoringPositions.put("D", new Pose2d(Units.inchesToMeters(165.877947035), Units.inchesToMeters(128.112777169),  new Rotation2d(Math.PI/3)      ));
        ReefScoringPositions.put("E", new Pose2d(Units.inchesToMeters(187.613142965), Units.inchesToMeters(128.112777169),  new Rotation2d(Math.PI*(2/3))  ));
        ReefScoringPositions.put("F", new Pose2d(Units.inchesToMeters(198.623492035), Units.inchesToMeters(134.469605169),  new Rotation2d(Math.PI*(2/3))  ));
        ReefScoringPositions.put("G", new Pose2d(Units.inchesToMeters(209.49109),     Units.inchesToMeters(153.292837),     new Rotation2d(Math.PI)        ));
        ReefScoringPositions.put("H", new Pose2d(Units.inchesToMeters(209.49109),     Units.inchesToMeters(166.006493),     new Rotation2d(Math.PI)        ));
        ReefScoringPositions.put("I", new Pose2d(Units.inchesToMeters(198.623492035), Units.inchesToMeters(184.829724831),  new Rotation2d(-Math.PI*(2/3)) ));
        ReefScoringPositions.put("J", new Pose2d(Units.inchesToMeters(187.613142965), Units.inchesToMeters(191.186552831),  new Rotation2d(-Math.PI*(2/3)) ));
        ReefScoringPositions.put("K", new Pose2d(Units.inchesToMeters(165.877947035), Units.inchesToMeters(191.186552831),  new Rotation2d(-Math.PI/3)     ));
        ReefScoringPositions.put("L", new Pose2d(Units.inchesToMeters(154.867597965), Units.inchesToMeters(184.829724831),  new Rotation2d(-Math.PI/3)     ));
    };

    /**
     * Flips the x, y, and rotation coordinates across the field, converting between field positions for the red and blue alliances
     * @param position the Pose2d position to be flipped
     * @return the flipped Pose2d
     */
    public static Pose2d flipPosition(Pose2d position) {
        return new Pose2d(Math.abs(Constants.FieldConstants.length - position.getX()), Math.abs(Constants.FieldConstants.width - position.getY()), position.getRotation().rotateBy(new Rotation2d(Math.PI)));
    }

    /**
     * Gets the Pose2d of the given scoring position
     * @param position letter value associated with one of the 12 scoring positions (A-K starting at the upper 9 o'clock position and moving CCW)
     * @return Pose2d or one of the scoring positions (against the base)
     */
    public Pose2d getReefScoringPosition(String position) {
        return ReefScoringPositions.get(position);
    }

    /**
     * Finds and returns the nearest reef scoring position (against the base)
     * @param currPos the robot's current positions
     * @return The Pose2d of the nearest scoring position
     */
    public Pose2d getNearestReefScoringPosition(Pose2d currPos) {
        return getNearestReefScoringPositionWithOffset(currPos, new Transform2d(0, 0, new Rotation2d(0)));
    };

    /**
     * Finds and returns the nearest reef scoring position (against the base) with an offset
     * @param currPos the robot's current positions
     * @param offset the offset by which the returned pose2d is tranformed
     * @return The Pose2d of the nearest scoring position
     */
    public Pose2d getNearestReefScoringPositionWithOffset(Pose2d currPos, Transform2d offset) {
        Pose2d nearestPos = currPos.nearest(new ArrayList<Pose2d>(ReefScoringPositions.values()));
        Pose2d nearestPosWithOffset = nearestPos.transformBy(offset);
        log.writeLogEcho(false, "Field", "getNearestReefScoringPositionWithOffset", currPos, nearestPos, offset, nearestPosWithOffset);
        //return (allianceSelection.getAlliance() == Alliance.Blue) ? nearestPosWithOffset : flipPosition(nearestPosWithOffset);
        return nearestPosWithOffset; //24-25 Coordinate system had 0,0 at the corner to the right of alliance drivers, rather than just at the right of the blue driver's corner
    };

}
