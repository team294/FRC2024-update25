// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utilities;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

/**
 * A class representing field coordinates.
 * <p> Field coordinates include:
 * <p> Robot X location in the field, in meters (0 = field edge in front of driver station, + = away from our drivestation)
 * <p> Robot Y location in the field, in meters (0 = right edge of field when standing in driver station, + = left when looking from our drivestation)
 * <p> Robot angle on the field (0 = facing away from our drivestation, + to the left, - to the right)
 */
public class Field {
    //Robot probably 31" with bumpers
    private final Rotation2d oneEighty = new Rotation2d(Math.PI);
    private final Rotation2d zero = new Rotation2d(0.0);
    private final Rotation2d twoSeventy = new Rotation2d(Math.PI*1.5);
    private final Rotation2d oneTwenty = new Rotation2d(Math.PI*(2/3));
    private final Rotation2d twoForty = new Rotation2d(Math.PI*(4/3));
    private final Rotation2d sixty = new Rotation2d(Math.PI/3);
    private final Rotation2d threeHundred = new Rotation2d(Math.PI*(5/3));
    private final Rotation2d oneTwentySix = new Rotation2d(Units.degreesToRadians(126));
    private final Rotation2d twoThirtyFour = new Rotation2d(Units.degreesToRadians(234));


    // BLUE  
    //IDs 1 and 2 are loading zone
    //ID 3 is amp
    //IDs 4 and 5 are speaker
    //IDs 6 - 8 are stage
    
    // RED
    //IDs 9 and 10 are loading zone
    //ID 11 is amp
    //IDs 12 and 13 are speaker
    //IDs 14 - 16 are stage
    private final AprilTag[] AprilTags = {
        new AprilTag(1, new Pose3d(new Pose2d(Units.inchesToMeters(657.37), Units.inchesToMeters(25.80), oneTwentySix))),      // 120 degrees
        };

    private final AllianceSelection alliance;
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
    public Field(AllianceSelection alliance, FileLog log){
        this.alliance = alliance;
        this.log = log;

        this.log.writeLogEcho(true, "Field", "Constructor", "Alliance", alliance.toString());
    }

    /**
	 * Gets the position of a specified April Tag (1-8)
     * <p> Note that the position will be different for red vs blue alliance, based on the current alliance in the alliance object.
	 * 
	 * @param position
	 */
    public AprilTag getAprilTag(int ID) throws IndexOutOfBoundsException {
        if(ID < 17 && ID > 0) {
            if(alliance.getAlliance() == Alliance.Blue) {
                return AprilTags[ID-1];
            } else {
                return AprilTags[ID-1];
            }
        } else {
            throw new IndexOutOfBoundsException(String.format("AprilTag ID %d out of range", ID));
        }
    }

    /**
     * gets alliance
     * @return Alliance current selected alliance color
     */
    public Alliance getAlliance() {
        return alliance.getAlliance();
    }

    /**
     * Need to check if april tag values match april tag on or to the side of the speaker
     * @return Speaker Pose2d Value
     */
    public Pose2d getSpeakerPose2d(){
        if(getAlliance() == Alliance.Blue){
            return getAprilTag(4).pose.toPose2d();
        } else {
            return getAprilTag(12).pose.toPose2d();
        }
    }
}
