// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utilities;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
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
        new AprilTag(1, new Pose3d(new Pose2d(15.0794638570895, 0.245871867229192, oneTwenty))),      // 120 degrees
        new AprilTag(2, new Pose3d(new Pose2d(16.1851252600324, 0.883665522820618, oneTwenty))),      // 120 degrees
        new AprilTag(3, new Pose3d(new Pose2d(1.84149900559054, 8.20419556973439, twoSeventy))),      // 270 degrees
        new AprilTag(4, new Pose3d(new Pose2d(-0.0380999794260111, 5.5478650041529, zero))),            // 0 degrees
        new AprilTag(5, new Pose3d(new Pose2d(-0.0380999794260111, 4.98271530933373, zero))),           // 0 degrees
        new AprilTag(6, new Pose3d(new Pose2d(5.32078912677387, 4.10514578322128, zero))),            // 0 degrees
        new AprilTag(7, new Pose3d(new Pose2d(4.64133949367667, 4.49833757089771, oneTwenty))),       // 120 degrees
        new AprilTag(8, new Pose3d(new Pose2d(4.64133949367667, 3.71322399485904, twoForty))),        // 240 degrees
        new AprilTag(9, new Pose3d(new Pose2d(0.356107807701784, 0.883665522820618, sixty))),         // 60 degrees
        new AprilTag(10, new Pose3d(new Pose2d(1.46151521078179, 0.245871867229192, sixty))),         // 60 degrees
        new AprilTag(11, new Pose3d(new Pose2d(14.700750061595, 8.20419556973439, twoSeventy))),      // 270 degrees
        new AprilTag(12, new Pose3d(new Pose2d(16.5793330471602, 4.98271530933373, oneEighty))),      // 180 degrees
        new AprilTag(13, new Pose3d(new Pose2d(16.5793330471602, 5.5478650041529, oneEighty))),       // 180 degrees
        new AprilTag(14, new Pose3d(new Pose2d(11.9047195714514, 3.71322399485904, threeHundred))),   // 300 degrees
        new AprilTag(15, new Pose3d(new Pose2d(11.9047195714514, 4.49833757089771, sixty))),          // 60 degrees
        new AprilTag(16, new Pose3d(new Pose2d(11.2201899410974, 4.10514578322128, oneEighty)))       // 180 degrees
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
