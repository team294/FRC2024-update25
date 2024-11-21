// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utilities;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.SwerveConstants;

/**
 * Class that defines and caches all trajectories that the robot could run.
 * Create one object instance of this class when the robot initializes to build the trajectories. 
 */
public class TrajectoryCache {
    private FileLog log;
   
    private static int trajectoryCount = 41;
    public TrajectoryFacingPair[] cache = new TrajectoryFacingPair[trajectoryCount];    // array of trajectories

    public enum TrajectoryType {
        test(0),
        driveToSourceCloseNote(1),
        driveToCenterCloseNote(2),
        driveToAmpCloseNote(3),
        driveAmpNoteToFarNote(4),
        driveSourceNoteToFarNote(5),
        driveCenterAmpNote(6),
        driveFromAmpToCenter(7),
        driveFromAmpNoteToCenterStart(8),
        driveFromCenterNoteToCenterStart(9),
        driveCenterToNearSource(10),
        driveFromSourceNearToCenterStart(11),
        driveFromCenterStartToEndCenterAuto(12),
        driveFromSourceNoteToSourceStart(13),
        driveCenterStartToSourceNear(14),
        driveFromSourceNoteToCenterNote(15),
        driveFromCenterNoteToAmpNote(16),
        driveSourceOutsideNotes(17),
        driveSourceOutsideNotestoCenterNote(18),
        driveCenterNotetoOutsideStage(19),
        driveOutsideStageLeftCenterNote(20),
        driveLeftCenterNotetoOutsideStage(21),
        driveFromAmpFarToShootingPos(22),
        driveAmpToFarCenter(23),
        driveFarCenterNoteToPodiumShot(24),
        drivePodiumShotToNextCenterNote(25),
        driveNextCenterNotetoPodiumShot(26),
        drivePodiumShotToCenterNote(27),
        driveCenterNotetoPodiumShot(28),
        driveAmpToSecondFarCenter(29),
        driveNextCenterNoteToCenterNote(30),
        driveFirstCenterAmpToNextCenterNote(31),
        driveFromAmpNoteToSecondCenter(32),
        driveFromCenterSecondToScorePos(33),
        driveSourceNextNoteToCenterNoteRight(34),
        drivePodiumShotToCenterRightNote(35),
        driveCenterRightNoteToPodiumShot(36),
        driveFromSourceToSideMobility(37),
        driveFromWaitSpotToShootingPos(38),
        driveFromSourceToWallMobility(39),
        driveAmpToFar2ndNote(40);



        @SuppressWarnings({"MemberName", "PMD.SingularField"})
        public final int value;
        TrajectoryType(int value) { this.value = value; }
    }

    /**
     * A trajectory with initial and final facing for the robot
     */
    public static class TrajectoryFacing {
        public final Rotation2d initialRotation, finalRotation;
        public final Trajectory trajectory;

        /**
         * Creates a trajectory with initial and final robot facing (rotation).
         * <p> Note that the trajectory by itself does *not* contain robot facings.  The Pose2d angles in the
         * trajectory are the direction of the velocity vector.
         * <p> This object adds initial and final facings for the trajectory.
         * @param initialRotation Expected facing (rotation) of robot at beginning of trajectory
         * @param finalRotation Desired facing (rotation) of robot at end of trajectory
         * @param trajectory The trajectory
         */
        public TrajectoryFacing(Rotation2d initialRotation, Rotation2d finalRotation, Trajectory trajectory) {
            this.initialRotation = initialRotation;
            this.finalRotation = finalRotation;
            this.trajectory = trajectory;
        }

        /**
         * Returns the intial robot pose (position and facing) for the robot prior to running the trajectory
         * @return initial Pose2d
         */
        public Pose2d getInitialPose() {
            return new Pose2d( trajectory.getInitialPose().getTranslation(), initialRotation);
        }
    }

    /**
     * A pair of trajectories with initial and final facings for the robot
     */
    public static class TrajectoryFacingPair {
        public final TrajectoryFacing red, blue;

        /**
         * Creates a pair of trajectories with initial and final robot facings based on a single trajectoryFacing
         * Used for creating "colorless" trajectories (not dependent on alliance)
         * @param trajectoryFacingColorless trajectoryFacing to use
         */
        public TrajectoryFacingPair(TrajectoryFacing trajectoryFacingColorless) {
            this(trajectoryFacingColorless, trajectoryFacingColorless);
        }

        /**
         * Creates a pair of trajectories with initial and final robot facings
         * @param trajectoryFacingRed   trajectoryFacing to use on the red alliance
         * @param trajectoryFacingBlue  trajectoryFacing to use on the blue alliance
         */
        public TrajectoryFacingPair(TrajectoryFacing trajectoryFacingRed, TrajectoryFacing trajectoryFacingBlue) {
            this.red = trajectoryFacingRed;
            this.blue = trajectoryFacingBlue;
        }
    }

    /**
     * Build all trajectories in this.cache[] for trajectory-following commands.
     * @param log
     */
    public TrajectoryCache(FileLog log){
        this.log = log;
        cache[TrajectoryType.test.value] = new TrajectoryFacingPair(
            new TrajectoryFacing(
                new Rotation2d(0),
                new Rotation2d(Math.PI/2),
                calcTrajectory("Test", .4, .4,
                new Pose2d(0, 0, new Rotation2d(Math.toRadians(0))),
                List.of(
                    new Translation2d(1, -(2 - Math.sqrt(3))),
                    new Translation2d(Math.sqrt(2), -(2 - Math.sqrt(2))),
                    new Translation2d(Math.sqrt(3), -1)
                ),
                new Pose2d(2, -2, new Rotation2d(Math.toRadians(270)))
                )
            )
        );

        cache[TrajectoryType.driveToSourceCloseNote.value] = new TrajectoryFacingPair(
            //Red Trajectory
            new TrajectoryFacing(
                new Rotation2d(Math.toRadians(54)), 
                new Rotation2d(0), 
                calcTrajectory("Source Start to Close Note Red", .4, .4, 
                    new Pose2d(1.1, 3.463, new Rotation2d(Math.toRadians(10))), 
                    List.of(), 
                    new Pose2d(2.8, 3.963, new Rotation2d(0))
                )
            ),
            //Blue Trajectory
            new TrajectoryFacing(
                new Rotation2d(Math.toRadians(-54)), 
                new Rotation2d(0), 
                calcTrajectory("Source Start to Close Note Blue", .4, .4, 
                    new Pose2d(1.3, 4.6, new Rotation2d(Math.toRadians(-10))), 
                    List.of(), 
                    new Pose2d(2.8, 4.1, new Rotation2d(0))
                )
            )
        );

        cache[TrajectoryType.driveToCenterCloseNote.value] = new TrajectoryFacingPair(
            //Red Trajectory
            new TrajectoryFacing(
                new Rotation2d(0), 
                new Rotation2d(0), 
                calcTrajectory("Center Start to near note Red", .4, .4, 
                    new Pose2d(1.3, 2.663, new Rotation2d(0)), 
                    List.of(), 
                    new Pose2d(3.1, 2.663, new Rotation2d(0))
                )
            ),
            //Blue Trajectory
            new TrajectoryFacing(new Rotation2d(0), 
                new Rotation2d(0), 
                calcTrajectory("Center Start to near note Blue", .4, .4, 
                    new Pose2d(1.3, 5.57, new Rotation2d(0)), 
                    List.of(), 
                    new Pose2d(3.1, 5.57, new Rotation2d(0))
                )
            )
        );
        
        cache[TrajectoryType.driveToAmpCloseNote.value] = new TrajectoryFacingPair(
            //Red Trajectory
            new TrajectoryFacing(
                new Rotation2d(Math.toRadians(-60)), 
                new Rotation2d(Math.toRadians(-26)), 
                calcTrajectory("Amp Start to Close Note Red", .8, .4, 
                    new Pose2d(0.7, 1.3, new Rotation2d(Math.toRadians(0))), 
                    List.of(), 
                    new Pose2d(3.0, 0.8, new Rotation2d(0))
                )
            ),
            //Blue Trajectory
            new TrajectoryFacing(
                new Rotation2d(Math.toRadians(60)), 
                new Rotation2d(Math.toRadians(26)), 
                calcTrajectory("Amp Start to Close Note Blue", .8, .4, 
                    new Pose2d(0.7, 6.5, new Rotation2d(Math.toRadians(0))), 
                    List.of(), 
                    new Pose2d(3.0, 7.4, new Rotation2d(0))
                )
            )
        );
        
        cache[TrajectoryType.driveAmpNoteToFarNote.value] = new TrajectoryFacingPair(
            //Red Trajectory
            new TrajectoryFacing(
                new Rotation2d(Math.toRadians(-25)), 
                new Rotation2d(0), 
                calcTrajectory("Drive Center To Far Note Red", .8, .8,
                    new Pose2d(2.0, 1.25, new Rotation2d(Math.toRadians(-30))), 
                    List.of(), 
                    new Pose2d(7.8, 0.75, new Rotation2d(0))
                )
            ),
            //Blue Trajectory
            new TrajectoryFacing(
                new Rotation2d(Math.toRadians(25)), 
                new Rotation2d(0), 
                calcTrajectory("Drive Center To Far Note Blue", .8, .8,
                    new Pose2d(2.0, 6.85, new Rotation2d(Math.toRadians(30))), 
                    List.of(), 
                    new Pose2d(7.8, 7.35, new Rotation2d(0))
                )
            )
        );

        cache[TrajectoryType.driveSourceNoteToFarNote.value] = new TrajectoryFacingPair(
            //Red Trajectory
            new TrajectoryFacing(
                new Rotation2d(0), 
                new Rotation2d(0), 
                calcTrajectory("Drive Center To Far Note Red", .4, .4,
                    new Pose2d(3.0, 4.1, new Rotation2d(Math.toRadians(-30))), 
                    List.of(), 
                    new Pose2d(8.2, 7.4, new Rotation2d(0))
                )
            ),
            //Blue Trajectory
            new TrajectoryFacing(
                new Rotation2d(0), 
                new Rotation2d(0), 
                calcTrajectory("Drive Center To Far Note Blue", .4, .4,
                    new Pose2d(3.0, 4.1, new Rotation2d(Math.toRadians(30))), 
                    List.of(), 
                    new Pose2d(8.2, 0.8, new Rotation2d(0))
                )
            )
        );

        cache[TrajectoryType.driveCenterAmpNote.value] = new TrajectoryFacingPair(
            //Red Trajectory
            new TrajectoryFacing(
                new Rotation2d(Math.toRadians(0)), 
                new Rotation2d(Math.toRadians(-33)), 
                calcTrajectory("Center Start to Amp Note Red", .8, .4, 
                    new Pose2d(1.3, 2.663, new Rotation2d(Math.toRadians(-33))), 
                    List.of(), 
                    new Pose2d(3.1, 1.22, new Rotation2d(Math.toRadians(-33)))
                )
            ),
            //Blue Trajectory
            new TrajectoryFacing(
                new Rotation2d(Math.toRadians(0)), 
                new Rotation2d(Math.toRadians(33)), 
                calcTrajectory("Center Start to Amp Note Blue", .8, .4, 
                    new Pose2d(1.3, 5.57, new Rotation2d(Math.toRadians(33))), 
                    List.of(), 
                    new Pose2d(3.1, 7.08, new Rotation2d(Math.toRadians(33)))
                )
            )
        );

        cache[TrajectoryType.driveFromAmpNoteToCenterStart.value] = new TrajectoryFacingPair(
            //Red Alliance
            new TrajectoryFacing(
                new Rotation2d(Math.toRadians(-33)), 
                new Rotation2d(Math.toRadians(0)), 
                calcTrajectory("Amp Note To Center Start Red", .8, .4, 
                    new Pose2d(3.1, 1.22, new Rotation2d(Math.toRadians(147))), 
                    List.of(), 
                    new Pose2d(1.3, 2.663, new Rotation2d(Math.toRadians(147)))
                )
            ),
            //Blue Alliance
            new TrajectoryFacing(
                new Rotation2d(Math.toRadians(33)), 
                new Rotation2d(Math.toRadians(0)), 
                calcTrajectory("Amp Note To Center Start Blue", .8, .4, 
                    new Pose2d(3.1, 7.08, new Rotation2d(Math.toRadians(-147))), 
                    List.of(), 
                    new Pose2d(1.3, 5.57, new Rotation2d(Math.toRadians(-147)))
                )
            )
        );

        cache[TrajectoryType.driveFromAmpToCenter.value] = new TrajectoryFacingPair(
            //Red Trajectory
            new TrajectoryFacing(
                new Rotation2d(Math.toRadians(-33)), 
                new Rotation2d(Math.toRadians(0)), 
                calcTrajectory("Amp Start to Close Note Red", .8, .4, 
                    new Pose2d(3.1, 1.6, new Rotation2d(Math.toRadians(147))), 
                    List.of(
                        new Translation2d(2.2, 2.7)
                    ), 
                    new Pose2d(3.1, 2.663, new Rotation2d(0))
                )
            ),
            //Blue Trajectory
            new TrajectoryFacing(
                new Rotation2d(Math.toRadians(33)), 
                new Rotation2d(Math.toRadians(0)), 
                calcTrajectory("Amp Start to Close Note Blue", .8, .4, 
                    new Pose2d(3.1, 7.5, new Rotation2d(Math.toRadians(-147))), 
                    List.of(
                        new Translation2d(2.2, 5.5)
                    ), 
                    new Pose2d(3.1, 5.57, new Rotation2d(0))
                )
            )
        );

        cache[TrajectoryType.driveFromCenterNoteToCenterStart.value] = new TrajectoryFacingPair(
            //Red Trajectory
            new TrajectoryFacing(
                new Rotation2d(0), 
                new Rotation2d(0), 
                calcTrajectory("Center Note to Center start Red", .4, .4, 
                    new Pose2d(3.1, 2.663, new Rotation2d(Math.toRadians(180))), 
                    List.of(), 
                    new Pose2d(1.3, 2.663, new Rotation2d(Math.toRadians(180)))
                )
            ),
            //Blue Trajectory
            new TrajectoryFacing(
                new Rotation2d(0), 
                new Rotation2d(0), 
                calcTrajectory("Center Note to Center Start Blue", .4, .4, 
                    new Pose2d(3.1, 5.57, new Rotation2d(Math.toRadians(180))), 
                    List.of(), 
                    new Pose2d(1.3, 5.57, new Rotation2d(Math.toRadians(180)))
                )
            )
        );
        
        cache[TrajectoryType.driveCenterToNearSource.value] = new TrajectoryFacingPair(
            //Red Trajectory
            new TrajectoryFacing(
                new Rotation2d(0), 
                new Rotation2d(Math.toRadians(33)), 
                calcTrajectory("Source Start to Close Note Red", .4, .4, 
                    new Pose2d(1.3, 2.663, new Rotation2d(Math.toRadians(33))), 
                    List.of(), 
                    new Pose2d(2.7, 4.1, new Rotation2d(Math.toRadians(33)))
                )
            ),
            //Blue Trajectory
            new TrajectoryFacing(
                new Rotation2d(0), 
                new Rotation2d(Math.toRadians(-33)), 
                calcTrajectory("Source Start to Close Note Blue", .4, .4, 
                    new Pose2d(1.3, 5.57, new Rotation2d(Math.toRadians(-33))), 
                    List.of(), 
                    new Pose2d(2.7, 4.1, new Rotation2d(Math.toRadians(-33)))
                )
            )
        );

        cache[TrajectoryType.driveFromSourceNearToCenterStart.value] = new TrajectoryFacingPair(
            //Red Trajectory
            new TrajectoryFacing(
                new Rotation2d(0), 
                new Rotation2d(Math.toRadians(33)), 
                calcTrajectory("Source Start to Close Note Red", .4, .4, 
                    new Pose2d(2.7, 4.1, new Rotation2d(Math.toRadians(-147))), 
                    List.of(), 
                    new Pose2d(1.3, 2.663, new Rotation2d(Math.toRadians(-147)))
                )
            ),
            //Blue Trajectory
            new TrajectoryFacing(
                new Rotation2d(0), 
                new Rotation2d(Math.toRadians(-33)), 
                calcTrajectory("Source Start to Close Note Blue", .4, .4, 
                    new Pose2d(2.7, 4.1, new Rotation2d(Math.toRadians(147))), 
                    List.of(), 
                    new Pose2d(1.3, 5.57, new Rotation2d(Math.toRadians(147)))
                )
            )
        );

        cache[TrajectoryType.driveFromCenterStartToEndCenterAuto.value] = new TrajectoryFacingPair(
            //Red Trajectory
            new TrajectoryFacing(
                new Rotation2d(0), 
                new Rotation2d(0), 
                calcTrajectory("Center Start to near note Red", .8, .4, 
                    new Pose2d(1.3, 2.663, new Rotation2d(Math.toRadians(-17))), 
                    List.of(), 
                    new Pose2d(7, 1.22, new Rotation2d(Math.toRadians(-17)))
                )
            ),
            //Blue Trajectory
            new TrajectoryFacing(
                new Rotation2d(0), 
                new Rotation2d(0), 
                calcTrajectory("Center Start to near note Blue", .8, .4, 
                    new Pose2d(1.3, 5.57, new Rotation2d(Math.toRadians(17))), 
                    List.of(), 
                    new Pose2d(7, 7.08, new Rotation2d(Math.toRadians(17)))
                )
            )
        );

        cache[TrajectoryType.driveFromSourceNoteToSourceStart.value] = new TrajectoryFacingPair(
            //Red Trajectory
            new TrajectoryFacing(
                new Rotation2d(0), 
                new Rotation2d(Math.toRadians(54)), 
                calcTrajectory("Source Note to Source Start Red", .8, .4, 
                    new Pose2d(2.8, 3.963, new Rotation2d(Math.toRadians(180))),
                    List.of(), 
                    new Pose2d(1.1, 3.463, new Rotation2d(Math.toRadians(-170)))
                )
            ),
            //Blue Trajectory
            new TrajectoryFacing(
                new Rotation2d(0), 
                new Rotation2d(Math.toRadians(-54)), 
                calcTrajectory("Source Note to Source Start Blue", .8, .4, 
                    new Pose2d(2.8, 4.1, new Rotation2d(Math.toRadians(180))), 
                    List.of(), 
                    new Pose2d(1.1, 4.6, new Rotation2d(Math.toRadians(170)))
                )
            )
        );
        
        cache[TrajectoryType.driveCenterStartToSourceNear.value] = new TrajectoryFacingPair(
            //Red Trajectory
            new TrajectoryFacing(
                new Rotation2d(0), 
                new Rotation2d(Math.toRadians(23)), 
                calcTrajectory("Center Start to Source note Red", .8, .8, 
                    new Pose2d(0.4, 2.65, new Rotation2d(Math.toRadians(23))), 
                    List.of(), 
                    new Pose2d(1.7, 3.95, new Rotation2d(Math.toRadians(23)))
                )
            ), 
            //Blue Trajectory
            new TrajectoryFacing(
                new Rotation2d(0), 
                new Rotation2d(Math.toRadians(-25)), 
                calcTrajectory("Center Start to Source note Blue", .8, .8, 
                    new Pose2d(0.4, 5.55, new Rotation2d(Math.toRadians(-23))), 
                    List.of(), 
                    new Pose2d(1.7, 4.25, new Rotation2d(Math.toRadians(-23)))
                )
            )
        );

        cache[TrajectoryType.driveFromSourceNoteToCenterNote.value] = new TrajectoryFacingPair(
            //Red Alliance
            new TrajectoryFacing(
                new Rotation2d(Math.toRadians(23)), 
                new Rotation2d(0), 
                calcTrajectory("Source Note to Center note Red", .8, .8, 
                    new Pose2d(1.7, 3.95, new Rotation2d(Math.toRadians(-160))),
                    List.of(new Translation2d(1.6, 2.65)), 
                    new Pose2d(2.0, 2.65, new Rotation2d(0))
                )
            ),
            //Blue Alliance
            new TrajectoryFacing(
                new Rotation2d(Math.toRadians(-25)), 
                new Rotation2d(0), 
                calcTrajectory("Source Note to Center note Blue", .8, .8, 
                    new Pose2d(1.7, 4.25, new Rotation2d(Math.toRadians(160))),
                    List.of(new Translation2d(1.6, 5.45)), 
                    new Pose2d(2.0, 5.45, new Rotation2d(0))
                )
            )
        );

        cache[TrajectoryType.driveFromCenterNoteToAmpNote.value] = new TrajectoryFacingPair(
            //Red Trajectory
            new TrajectoryFacing(
                new Rotation2d(0), 
                new Rotation2d(Math.toRadians(-25)), 
                calcTrajectory("Center Note to Amp note Red", .8, .8, 
                    new Pose2d(2.0, 2.65, new Rotation2d(Math.toRadians(-160))),
                    List.of(new Translation2d(1.6, 1.25)), 
                    new Pose2d(2.0, 1.25, new Rotation2d(Math.toRadians(0)))
                )
            ),
            //Blue Trajectory
            new TrajectoryFacing(
                new Rotation2d(0), 
                new Rotation2d(Math.toRadians(25)), 
                calcTrajectory("Center Note  to Amp note Blue", .8, .8, 
                    new Pose2d(2.0, 5.45, new Rotation2d(Math.toRadians(160))),
                    List.of(new Translation2d(1.6, 6.85)), 
                    new Pose2d(2.0, 6.85, new Rotation2d(Math.toRadians(0)))
                )
            )
        );
        
        cache[TrajectoryType.driveSourceOutsideNotes.value] = new TrajectoryFacingPair(
            //Red Trajectory
            new TrajectoryFacing(
                new Rotation2d(Math.toRadians(54)), 
                new Rotation2d(0), 
                calcTrajectory("Source to next to note Red", .9, .9, 
                    new Pose2d(0.8, 3.73, new Rotation2d(Math.toRadians(54))), 
                    List.of(new Translation2d(1.2, 4.3)), 
                    new Pose2d(3.5, 5.705, new Rotation2d(0))
                )
            ),
            //Blue Trajectory
            new TrajectoryFacing(
                new Rotation2d(Math.toRadians(-54)), 
                new Rotation2d(Math.toRadians(0)), 
                calcTrajectory("Source to next to note Blue", .9, .9, 
                    new Pose2d(0.8, 4.5, new Rotation2d(Math.toRadians(-54))), 
                    List.of(), 
                    new Pose2d(3.5, 2.5246, new Rotation2d(0)) 
                )
            )
        );
        
        cache[TrajectoryType.driveSourceOutsideNotestoCenterNote.value] = new TrajectoryFacingPair(
            //Red Trajectory
            new TrajectoryFacing(
                new Rotation2d(0), 
                new Rotation2d(0), 
                calcTrajectory("Next to note to center note Red", .8, .8, 
                    new Pose2d(3.5, 5.705, new Rotation2d(0)),
                    List.of(
                        new Translation2d(4.8, 4.1436),
                        new Translation2d(6.0592, 4.1436)
                    ), 
                    new Pose2d(7.9, 4.1546, new Rotation2d(0))
                )
            ),
            //Blue Trajectory
            new TrajectoryFacing(
                new Rotation2d(0), 
                new Rotation2d(0), 
                calcTrajectory("Next to note to center note Blue", .8, .8, 
                    new Pose2d(3.5, 2.5246, new Rotation2d(0)),
                    List.of(
                        new Translation2d(4.8, 4.086),
                        new Translation2d(6.0592, 4.086)
                    ), 
                    new Pose2d(7.9, 4.075, new Rotation2d(0))
                )
            )
        );

        cache[TrajectoryType.driveCenterNotetoOutsideStage.value] = new TrajectoryFacingPair(
            //Red Trajectory
            new TrajectoryFacing(
                new Rotation2d(0), 
                new Rotation2d(0), 
                calcTrajectory("Center Note to podium shot Red", .8, .8, 
                    new Pose2d(7.9, 4.1546, new Rotation2d(Math.PI)),
                    List.of(
                        new Translation2d(6.0592, 4.1436),
                        new Translation2d(4.8, 4.1436)
                    ), 
                    new Pose2d(3.7582, 2.7516, new Rotation2d(Math.PI))
                )
            ),
            //Blue Trajectory
            new TrajectoryFacing(
                new Rotation2d(0), 
                new Rotation2d(0), 
                calcTrajectory("Center Note to podium shot Blue", .8, .8, 
                    new Pose2d(7.9, 4.075, new Rotation2d(Math.PI)),
                    List.of(
                        new Translation2d(6.0592, 4.086),
                        new Translation2d(4.8, 4.086)
                    ), 
                    new Pose2d(3.7582, 5.478, new Rotation2d(Math.PI))
                )
            )
        );

        cache[TrajectoryType.driveOutsideStageLeftCenterNote.value] = new TrajectoryFacingPair(
            //Red Trajectory
            new TrajectoryFacing(
                new Rotation2d(0), 
                new Rotation2d(0), 
                calcTrajectory("Podium shot to center left note Red", .8, .8,
                    new Pose2d(3.7582, 2.7516, new Rotation2d(0)),
                    List.of(
                        new Translation2d(4.8, 4.1436),
                        new Translation2d(6.0592, 4.1436)
                    ), 
                    new Pose2d(7.9, 5.7986, new Rotation2d(0))
                )
            ),
            //Blue Trajectory
            new TrajectoryFacing(
                new Rotation2d(0), 
                new Rotation2d(0), 
                calcTrajectory("Podium shot to center left note Blue", .8, .8,
                    new Pose2d(3.7582, 5.478, new Rotation2d(0)),
                    List.of(
                        new Translation2d(4.8, 4.086),
                        new Translation2d(6.0592, 4.086)
                    ), 
                    new Pose2d(7.9, 2.431, new Rotation2d(0))
                )
            )
        );
    
        cache[TrajectoryType.driveLeftCenterNotetoOutsideStage.value] = new TrajectoryFacingPair(
            //Red Trajectory
            new TrajectoryFacing(
                new Rotation2d(0), 
                new Rotation2d(0), 
                calcTrajectory("Left Center note to podium shot Red", .8, .8, 
                    new Pose2d(7.9, 5.7986, new Rotation2d(Math.PI)),
                    List.of(
                        new Translation2d(6.0592, 4.1436),
                        new Translation2d(4.8, 4.1436)
                    ), 
                    new Pose2d(3.7582, 2.7516, new Rotation2d(Math.PI))
                )
            ),
            //Blue Trajectory
            new TrajectoryFacing(
                new Rotation2d(0), 
                new Rotation2d(0), 
                calcTrajectory("Left Center note to podium shot Blue", .8, .8, 
                    new Pose2d(7.9, 2.431, new Rotation2d(Math.PI)),
                    List.of(
                        new Translation2d(6.0592, 4.086),
                        new Translation2d(4.8, 4.086)
                    ), 
                    new Pose2d(3.7582, 5.478, new Rotation2d(Math.PI))
                )
            )
        );

        cache[TrajectoryType.driveFromAmpFarToShootingPos.value] = new TrajectoryFacingPair(
            //Red Trajectory
            new TrajectoryFacing(
                new Rotation2d(Math.toRadians(0)), 
                new Rotation2d(0), 
                calcTrajectory("Drive Amp Far Note To shooting pos Red", .9, .9,
                    new Pose2d(7.8, 0.75, new Rotation2d(Math.toRadians(180))), 
                    List.of(new Translation2d(5, 1)), 
                    new Pose2d(3.5, 2.65, new Rotation2d(Math.toRadians(120)))
                )
            ),
            //Blue Trajectory
            new TrajectoryFacing(
                new Rotation2d(Math.toRadians(0)), 
                new Rotation2d(0), 
                calcTrajectory("Drive Amp Far Note To shooting pos Blue", .9, .9,
                    new Pose2d(7.8, 7.35, new Rotation2d(Math.toRadians(180))), 
                    List.of(new Translation2d(5, 7.1)), 
                    new Pose2d(3.5, 5.45, new Rotation2d(Math.toRadians(-120)))
                )
            )
        );
        
        cache[TrajectoryType.driveAmpToFarCenter.value] = new TrajectoryFacingPair(
            //Red Trajectory
            new TrajectoryFacing(
                new Rotation2d(Math.toRadians(-60)), 
                new Rotation2d(0), 
                calcTrajectory("Amp to Far center note Red", .9, .8, 
                    new Pose2d(0.8, 1.6296, new Rotation2d(Math.toRadians(-60))), 
                    List.of(
                        new Translation2d(2.6, 0.42)            // changed from 0.47 to 0.42 to avoid hitting note
                    ), 
                    new Pose2d(9, 0.7696, new Rotation2d(0))
                )
            ),
            //Blue Trajectory
            new TrajectoryFacing(
                new Rotation2d(Math.toRadians(60)), 
                new Rotation2d(0), 
                calcTrajectory("Amp to Far center note Blue", .9, .8, 
                    new Pose2d(0.8, 6.6, new Rotation2d(Math.toRadians(60))), 
                    List.of(new Translation2d(2.6, 7.8096)),   // changed from 7.7596 to 7.8096
                    new Pose2d(9, 7.460, new Rotation2d(0)) 
                )
            )
        );

        cache[TrajectoryType.driveAmpToFar2ndNote.value] = new TrajectoryFacingPair(
            //Red Trajectory
            new TrajectoryFacing(
                new Rotation2d(Math.toRadians(-60)), 
                new Rotation2d(0), 
                calcTrajectory("Amp to Far second note Red", .9, .8, 
                    new Pose2d(0.8, 1.6296, new Rotation2d(Math.toRadians(-60))), 
                    List.of(
                        new Translation2d(2.6, 0.42),        
                        new Translation2d(6, 1.6)            // F10 changed from 4.5 to 6 to avoid hitting wall
                    ), 
                    new Pose2d(9, 2.4296 + 0.3556, new Rotation2d(0))
                )
            ),
            //Blue Trajectory
            new TrajectoryFacing(
                new Rotation2d(Math.toRadians(60)), 
                new Rotation2d(0), 
                calcTrajectory("Amp to Far second note Blue", .9, .8, 
                    new Pose2d(0.8, 6.6, new Rotation2d(Math.toRadians(60))),
                    List.of(
                        new Translation2d(2.6, 7.8096),
                        new Translation2d(6, 7.8096 - 1.18)      // F10 changed from 4.5 to 6 to avoid hitting wall
                    ),   
                    new Pose2d(9, 5.8 - 0.3556, new Rotation2d(0))
                )
            )
        );
            
        cache[TrajectoryType.driveFarCenterNoteToPodiumShot.value] = new TrajectoryFacingPair(
            //Red Trajectory
            new TrajectoryFacing(
                new Rotation2d(0), 
                new Rotation2d(Math.toRadians(-10)), 
                calcTrajectory("Far Center note to Podium Shot Red", .9, .8, 
                new Pose2d(9, 0.7696, new Rotation2d(Math.PI)),
                List.of(), 
                new Pose2d(4.6, 1.9296, new Rotation2d(Math.PI)) 
                )
            ),
            //Blue Trajectory
            new TrajectoryFacing(
                new Rotation2d(0), 
                new Rotation2d(Math.toRadians(10)), 
                calcTrajectory("Center note to Podium Shot Blue", .9, .8, 
                new Pose2d(9, 7.46, new Rotation2d(Math.PI)), 
                List.of(), 
                new Pose2d(4.6, 6.3, new Rotation2d(Math.PI)) 
                )
            )
        );

       cache[TrajectoryType.drivePodiumShotToNextCenterNote.value] = new TrajectoryFacingPair(
            //Red Trajectory
            new TrajectoryFacing(
                new Rotation2d(Math.toRadians(-10)), 
                new Rotation2d(0), 
                calcTrajectory("Podium Shot to Next Center note Red", .9, .8, 
                    new Pose2d(4.6, 1.9296, new Rotation2d(0)), 
                    List.of(new Translation2d(7, 2.1)), 
                    new Pose2d(9, 2.4296 + 0.3556, new Rotation2d(0)) //F8: Added 0.3556
                )
            ),
            //Blue Trajectory
            new TrajectoryFacing(
                new Rotation2d(Math.toRadians(10)), 
                new Rotation2d(0), 
                calcTrajectory("Podium Shot to Next Center note Blue", .9, .8, 
                    new Pose2d(4.6, 6.3, new Rotation2d(0)), 
                    List.of(new Translation2d(7, 6.1296)), 
                    new Pose2d(9, 5.8-0.3556, new Rotation2d(0))  //F8: Subtracted 0.3556
                )
            )
       );

       cache[TrajectoryType.driveNextCenterNotetoPodiumShot.value] = new TrajectoryFacingPair(
            //Red Trajectory
            new TrajectoryFacing(
                new Rotation2d(0), 
                new Rotation2d(Math.toRadians(-13)), // F5 Changed from -10 to -13
                calcTrajectory("Next Center note to Podium Shot Red", .9, .8, 
                    new Pose2d(9, 2.4296 +0.3556, new Rotation2d(Math.PI)), //F8: Added 0.3556
                    List.of(new Translation2d(7, 2.1)), 
                    new Pose2d(4.6, 1.9296, new Rotation2d(Math.PI))
                )
            ),
            //Blue Trajectory
            new TrajectoryFacing(
                new Rotation2d(0), 
                new Rotation2d(Math.toRadians(13)), // F5 Changed from 10 to 13
                calcTrajectory("Next Center note to Podium Shot Blue", .9, .8, 
                    new Pose2d(9, 5.8 -0.3556, new Rotation2d(Math.PI)), //F8: Subtracted 0.3556
                    List.of(new Translation2d(7, 6.1296)), 
                    new Pose2d(4.6, 6.3, new Rotation2d(Math.PI))
                )
            )
        );

      cache[TrajectoryType.drivePodiumShotToCenterNote.value] = new TrajectoryFacingPair(
            //Red Trajectory
            new TrajectoryFacing(
                new Rotation2d(Math.toRadians(-13)), // F5 Changed from -10 to -13
                new Rotation2d(Math.toRadians(20)), 
                calcTrajectory("Podium Shot to Center note Red", .9, .8, 
                    new Pose2d(4.6, 1.929, new Rotation2d(0)), 
                    List.of(new Translation2d(6.647, 2.4946)), 
                    new Pose2d(8.8, 4.0996 +.3556, new Rotation2d(0)) //F8: Added 0.3556
                )
            ),
            //Blue Trajectory
            new TrajectoryFacing(
                new Rotation2d(Math.toRadians(13)), // F5 Changed from 10 to 13
                new Rotation2d(Math.toRadians(-20)), 
                calcTrajectory("Podium Shot to Center note Blue", .9, .8, 
                    new Pose2d(4.6, 6.3, new Rotation2d(0)), 
                    List.of(new Translation2d(6.647, 5.735)), 
                    new Pose2d(8.8, 4.13 -.3556, new Rotation2d(0)) //F8: Subtracted 0.3556
                )
            )
        );

        cache[TrajectoryType.driveCenterNotetoPodiumShot.value] = new TrajectoryFacingPair(
            //Red Trajectory
            new TrajectoryFacing(
                new Rotation2d(Math.toRadians(20)), 
                new Rotation2d(0), 
                calcTrajectory("Center note to Podium Shot Red", .9, 8, 
                    new Pose2d(8.8, 4.0996 +0.3556, new Rotation2d(Math.PI)), //F8: Added 0.3556
                    List.of(new Translation2d(4.9, 4.0296)), 
                    new Pose2d(3.7582, 2.7516, new Rotation2d(Math.PI)) 
                )
            ),
            //Blue Trajectory
            new TrajectoryFacing(
                new Rotation2d(Math.toRadians(-20)), 
                new Rotation2d(0), 
                calcTrajectory("Center note to Podium Shot Blue", .9, .8, 
                    new Pose2d(8.8, 4.13 -0.3556, new Rotation2d(Math.PI)), //F8: Subtracted 0.3556
                    List.of(new Translation2d(4.9, 4.2)), 
                    new Pose2d(3.7582, 5.478, new Rotation2d(Math.PI)) 
                )
            )
        );            
    
            

        cache[TrajectoryType.driveAmpToSecondFarCenter.value] = new TrajectoryFacingPair(
            //Red Trajectory
            new TrajectoryFacing(
                new Rotation2d(Math.toRadians(-60)), 
                new Rotation2d(0), 
                calcTrajectory("Amp to Second Far center note Red", .9, .8, 
                    new Pose2d(0.8, 1.6296, new Rotation2d(Math.toRadians(-60))), 
                    List.of(new Translation2d(2.6, 0.47)), 
                    new Pose2d(9, 2.4296, new Rotation2d(0))
                )
            ),
            //Blue Trajectory
            new TrajectoryFacing(
                new Rotation2d(Math.toRadians(60)), 
                new Rotation2d(0), 
                calcTrajectory("Amp to Second Far center note Blue", .9, .8, 
                    new Pose2d(0.8, 6.6, new Rotation2d(Math.toRadians(54))), 
                    List.of(new Translation2d(2.6, 7.700)), 
                    new Pose2d(9, 5.8, new Rotation2d(0)) 
                )
            )
        );
        
        cache[TrajectoryType.driveNextCenterNoteToCenterNote.value] = new TrajectoryFacingPair(
            //Red Trajectory
            new TrajectoryFacing(
                new Rotation2d(0), 
                new Rotation2d(Math.toRadians(20)), 
                calcTrajectory("Next center note to middle note red", .9, .25, 
                    new Pose2d(9, 2.4296 +.3556, new Rotation2d(Math.PI)), //F8: Added 0.3556
                    List.of(new Translation2d(7.2, 3.25)), 
                    new Pose2d(8.8, 3.95 +.3556, new Rotation2d(Math.toRadians(20))) //F8: Added 0.3556
                )
            ),
            //Blue Trajectory
            new TrajectoryFacing(
                new Rotation2d(0), 
                new Rotation2d(Math.toRadians(-20)), 
                calcTrajectory("Next center note to middle note blue", .9, .25, 
                    new Pose2d(9, 5.8 -.3556, new Rotation2d(Math.PI)), //F8: Subtracted 0.3556
                    List.of(new Translation2d(7.2, 4.9796)), 
                    new Pose2d(8.8, 4.2796 -.3556, new Rotation2d(Math.toRadians(-20))) //F8: Subtracted 0.3556
                )
            )
        );

        cache[TrajectoryType.driveFirstCenterAmpToNextCenterNote.value] = new TrajectoryFacingPair(
            //Red Trajectory
            new TrajectoryFacing(
                new Rotation2d(0), 
                new Rotation2d(0), 
                calcTrajectory("First center note to next middle note red", .9, .25, 
                    new Pose2d(9, 0.7696, new Rotation2d(Math.PI)),
                    List.of(new Translation2d(7.2, 2)), 
                    new Pose2d(9, 2.4296 +.3556, new Rotation2d(0)) //F8: Added 0.3556
                )
            ),
            //Blue Trajectory
            new TrajectoryFacing(
                new Rotation2d(0), 
                new Rotation2d(0), 
                calcTrajectory("First center note to next middle note blue", .9, .25, 
                    new Pose2d(9, 7.460, new Rotation2d(Math.PI)), 
                    List.of(new Translation2d(7.2, 6.2296)), 
                    new Pose2d(9, 5.8 -0.3556, new Rotation2d(0)) //F8: Subtracted 0.3556
                )
            )
        );

        cache[TrajectoryType.driveFromAmpNoteToSecondCenter.value] = new TrajectoryFacingPair(
            //Red Trajectory
            new TrajectoryFacing(
                new Rotation2d(Math.toRadians(-25)), 
                new Rotation2d(0), 
                calcTrajectory("Drive Amp Note To Second Far Note Red", .9, .9,
                    new Pose2d(2.0, 1.25, new Rotation2d(Math.toRadians(0))), 
                    List.of(new Translation2d(5, 1.25)), 
                    new Pose2d(7.8, 2.4, new Rotation2d(0))
                )
            ),
            //Blue Trajectory
            new TrajectoryFacing(
                new Rotation2d(Math.toRadians(25)), 
                new Rotation2d(0), 
                calcTrajectory("Drive Center To Second Far Note Blue", .9, .9,
                    new Pose2d(2.0, 6.85, new Rotation2d(Math.toRadians(0))), 
                    List.of(new Translation2d(5, 6.85)), 
                    new Pose2d(7.8, 5.7, new Rotation2d(0))
                )
            )
        );

        cache[TrajectoryType.driveFromCenterSecondToScorePos.value] = new TrajectoryFacingPair(
            //Red Trajectory
            new TrajectoryFacing(
                new Rotation2d(Math.toRadians(0)), 
                new Rotation2d(0), 
                calcTrajectory("Drive Second Amp Far Note To shooting pos Red", .9, .9,
                    new Pose2d(7.8, 2.4, new Rotation2d(Math.toRadians(180))), 
                    List.of(new Translation2d(5, 1.3)), 
                    new Pose2d(3.5, 2.65, new Rotation2d(Math.toRadians(120)))
                )
            ),
            //Blue Trajectory
            new TrajectoryFacing(
                new Rotation2d(Math.toRadians(0)), 
                new Rotation2d(0), 
                calcTrajectory("Drive Second Amp Far Note To shooting pos Blue", .9, .9,
                    new Pose2d(7.8, 5.7, new Rotation2d(Math.toRadians(180))), 
                    List.of(new Translation2d(5, 6.8)), 
                    new Pose2d(3.5, 5.45, new Rotation2d(Math.toRadians(-120)))
                )
            )
        );

        cache[TrajectoryType.driveSourceNextNoteToCenterNoteRight.value] = new TrajectoryFacingPair(
            //Red Trajectory
            new TrajectoryFacing(
                new Rotation2d(0), 
                new Rotation2d(0), 
                calcTrajectory("Next to note to right center note Red", .8, .8, 
                    new Pose2d(3.5, 5.705, new Rotation2d(0)),
                    List.of(
                        new Translation2d(5.5, 6.4296)
                    ), 
                    new Pose2d(7.9, 5.7986, new Rotation2d(0))
                )
            ),
            //Blue Trajectory
            new TrajectoryFacing(
                new Rotation2d(0), 
                new Rotation2d(0), 
                calcTrajectory("Next to note to right center note Blue", .8, .8, 
                    new Pose2d(3.5, 2.5246, new Rotation2d(0)),
                    List.of(
                        new Translation2d(5.5, 1.8)
                    ), 
                    new Pose2d(7.9, 2.431, new Rotation2d(0))
                )
            )
        );
    
        cache[TrajectoryType.drivePodiumShotToCenterRightNote.value] = new TrajectoryFacingPair(
            //Red Trajectory
            new TrajectoryFacing(
                new Rotation2d(0), 
                new Rotation2d(0), 
                calcTrajectory("Podium shot to center right note Red", .8, .8, 
                    new Pose2d(3.7582, 2.7516, new Rotation2d(0)),
                    List.of(
                        new Translation2d(4.8, 4.1436),   
                        new Translation2d(6.0592, 4.1436)    
                    ), 
                    new Pose2d(7.9, 2.431, new Rotation2d(0))
                )
            ),
            //Blue Trajectory
            new TrajectoryFacing(
                new Rotation2d(0), 
                new Rotation2d(0), 
                calcTrajectory("Podium shot to center right note Blue", .8, .8, 
                    new Pose2d(3.7582, 5.478, new Rotation2d(0)),
                    List.of(
                        new Translation2d(4.8, 4.086),
                        new Translation2d(6.0592, 4.086) 

                    ), 
                    new Pose2d(7.9, 5.7986, new Rotation2d(0))
                )
            )
        );

        cache[TrajectoryType.driveCenterRightNoteToPodiumShot.value] = new TrajectoryFacingPair(
            //Red Trajectory
            new TrajectoryFacing(
                new Rotation2d(0), 
                new Rotation2d(0), 
                calcTrajectory("Center right note to Podium Red", .8, .8, 
                    new Pose2d(7.9, 2.431, new Rotation2d(Math.PI)),
                    List.of(
                        new Translation2d(6.0592, 4.1436),
                        new Translation2d(4.8, 4.1436) 
                    ), 
                    new Pose2d(3.7582, 2.7516, new Rotation2d(Math.PI))
                )
            ), 
            //Blue Trajectory
            new TrajectoryFacing(
                new Rotation2d(0), 
                new Rotation2d(0), 
                calcTrajectory("Center right note to Podium Blue", .8, .8, 
                    new Pose2d(7.9, 5.7986, new Rotation2d(Math.PI)),
                    List.of(
                        new Translation2d(6.0592, 4.086),
                        new Translation2d(4.8, 4.086) 
                    ), 
                    new Pose2d(3.7582, 5.478, new Rotation2d(Math.PI))
                )
            )
        );           
                
            
        
        cache[TrajectoryType.driveFromSourceToSideMobility.value] = new TrajectoryFacingPair(
            //Red Trajectory
            new TrajectoryFacing(
                new Rotation2d(Math.toRadians(60)), 
                new Rotation2d(0), 
                calcTrajectory("Source Start To far note Red", .8, .8, 
                    new Pose2d(0.8, 3.7296, new Rotation2d(Math.toRadians(90))),
                    List.of(
                        new Translation2d(1.4, 6.8296)
                    ), 
                    new Pose2d(9, 7.460, new Rotation2d(0))
                )
            ),
            //Blue Trajectory
            new TrajectoryFacing(
                new Rotation2d(Math.toRadians(-60)), 
                new Rotation2d(0), 
                calcTrajectory("Source Start to far note Blue", .8, .8, 
                    new Pose2d(0.8, 4.5, new Rotation2d(Math.toRadians(-90))),
                    List.of(
                        new Translation2d(1.4, 1.4)
                    ), 
                    new Pose2d(9, 0.7696, new Rotation2d(0))
                )
            )
        );

        cache[TrajectoryType.driveFromWaitSpotToShootingPos.value] = new TrajectoryFacingPair(
            //Red Trajectory
            new TrajectoryFacing(
                new Rotation2d(0), 
                new Rotation2d(Math.toRadians(60)), 
                calcTrajectory("Source Center Note To Start Red", .8, .8, 
                    new Pose2d(8, 7.460, new Rotation2d(Math.PI)),
                    List.of(
                        new Translation2d(1.4, 6.8296)
                    ), 
                    new Pose2d(0.8, 3.7296, new Rotation2d(Math.toRadians(-90)))
                )
            ),
            //Blue Trajectory
            new TrajectoryFacing(
                new Rotation2d(0), 
                new Rotation2d(Math.toRadians(-60)), 
                calcTrajectory("Source Center Note To Start Blue", .8, .8, 
                    new Pose2d(8, 0.7696, new Rotation2d(Math.PI)),
                    List.of(
                        new Translation2d(1.4, 1.4)
                    ), 
                    new Pose2d(0.8, 4.5, new Rotation2d(Math.toRadians(90)))
                )
            )
        );

        cache[TrajectoryType.driveFromSourceToWallMobility.value] = new TrajectoryFacingPair(
            //Red Trajectory
            new TrajectoryFacing(
                new Rotation2d(Math.toRadians(60)), 
                new Rotation2d(0), 
                calcTrajectory("Source Start To far note Red", .8, .8, 
                    new Pose2d(0.8, 3.7296, new Rotation2d(Math.toRadians(90))),
                    List.of(
                        new Translation2d(1.4, 6.8296)
                    ), 
                    new Pose2d(3, 7.460, new Rotation2d(0))
                )
            ),
            //Blue Trajectory
            new TrajectoryFacing(
                new Rotation2d(Math.toRadians(-60)), 
                new Rotation2d(0), 
                calcTrajectory("Source Start to far note Blue", .8, .8, 
                    new Pose2d(0.8, 4.5, new Rotation2d(Math.toRadians(-90))),
                    List.of(
                        new Translation2d(1.4, 1.4)
                    ), 
                    new Pose2d(3, 0.7696, new Rotation2d(0))
                )
            )
        );
            

        
    }


    /**
     * Builds a single trajectory based on the parameters passed in:
     * <p> Note that the trajectory by itself does *not* contain robot facings.  The Pose2d angles in the
     * trajectory are the direction of the velocity vector.
     * @param trajName name of the trajectory
     * @param maxVelRatio maximum velocity multiplier between 0 and 1
     * @param maxAccelRatio maximum acceleration multiplier between 0 and 1
     * @param startPose Pose2d starting position (coordinates and angle)
     * @param interriorWaypoints List of Translation 2d waypoints (just coordinates)
     * @param endPose Pose2d ending position (coordinates and angle)
     * @return trajectory that is generated
     */
    private Trajectory calcTrajectory(String trajName, double maxVelRatio, double maxAccelRatio, 
        Pose2d startPose, List<Translation2d> interriorWaypoints, Pose2d endPose) {
		Trajectory trajectory = null;
	
    	try {

			log.writeLogEcho(true, "TrajectoryGeneration", trajName, 
				"maxSpeed", SwerveConstants.kFullSpeedMetersPerSecond * maxVelRatio,
				"maxAcceleration", SwerveConstants.kFullAccelerationMetersPerSecondSquare * maxAccelRatio);

			// Create config for trajectory
            TrajectoryConfig config = new TrajectoryConfig(SwerveConstants.kFullSpeedMetersPerSecond * maxVelRatio,
				SwerveConstants.kFullAccelerationMetersPerSecondSquare * maxAccelRatio)
				.setKinematics(DriveConstants.kDriveKinematics);

            // Generate the trajectory
			trajectory = TrajectoryGenerator.generateTrajectory(
				startPose, interriorWaypoints, endPose, config);

			// debug logging
			TrajectoryUtil.dumpTrajectory(trajectory, log);

		} catch (Exception e) {
			log.writeLogEcho(true, "TrajectoryGeneration", trajName, 
				"ERROR in calcTrajectory", e.toString(),"exception",e);
		}

		if (trajectory != null) {
			log.writeLogEcho(true, "TrajectoryGeneration", trajName, "SUCCESS", true);
		};
	
		return trajectory;
	}

}