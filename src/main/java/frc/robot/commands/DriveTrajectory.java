package frc.robot.commands;

import java.util.Optional;

import frc.robot.Constants;
import frc.robot.Constants.CoordType;
import frc.robot.Constants.StopType;
import frc.robot.subsystems.DriveTrain;
import frc.robot.utilities.AllianceSelection;
import frc.robot.utilities.FileLog;
import frc.robot.utilities.TrajectoryCache.TrajectoryFacing;
import frc.robot.utilities.TrajectoryCache.TrajectoryFacingPair;
import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;


public class DriveTrajectory extends SequentialCommandGroup { 

    // Save initial pose for relative trajectories
    private Pose2d initialPose;

    /**
     * Follows a trajectory with swerve drive.
     * @param trajectoryType Specify what robot starting position to use
     * kRelative = path starts where robot is, kAbsolute = path starts where it was told to regardless of whether the robot is actually there
     * kAbsoluteResetPose = path starts where it was told to and robot is set at that starting point/facing,
     * kAbsoluteResetPoseTol = path starts where it was told to and robot is set at that starting point/facing if it is not already close to that facing
     * @param stopAtEnd  True = robot stops at end of trajectory, False = robot does not end stopped
     * @param trajectoryFacing The trajectory to follow and the final facing of the robot.  Note that
     * the trajectoryFacing.initialRotation is *ignored*.
     * @param driveTrain The driveTrain subsystem to be controlled.
     * @param log        File for logging
     */
    public DriveTrajectory(CoordType trajectoryType, StopType stopAtEnd, TrajectoryFacing trajectoryFacing, DriveTrain driveTrain, FileLog log) { 
        // Log that the command started
        addCommands(new FileLogWrite(false, false, "DriveTrajectory", "Start", log));

        // Define the controller for robot rotation
        ProfiledPIDController thetaController =
            new ProfiledPIDController(
                Constants.TrajectoryConstants.kPThetaController, 0, 0, Constants.TrajectoryConstants.kThetaControllerConstraints);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        // Create the serve controller command to follow the trajectory
        // Also add any initial commands before following the trajectory, depending on trajectoryType
        SwerveControllerLogCommand swerveControllerLogCommand;
        if (trajectoryType == CoordType.kRelative) {
            // For relative trajectories, first command needs to be to save the initial robot Pose
            addCommands(new InstantCommand(() -> initialPose = driveTrain.getPose()));

            swerveControllerLogCommand =
                new SwerveControllerLogCommand(
                    trajectoryFacing.trajectory,
                    // For relative trajectories, get the current pose relative to the initial robot Pose
                    () -> driveTrain.getPose().relativeTo(initialPose),  
                    Constants.DriveConstants.kDriveKinematics,
                    new PIDController(Constants.TrajectoryConstants.kPXController, 0, 0),
                    new PIDController(Constants.TrajectoryConstants.kPYController, 0, 0),
                    thetaController,
                    () -> trajectoryFacing.finalRotation,
                    (a) -> driveTrain.setModuleStates(a, false),
                    log,
                    driveTrain);
        } else {
            if (trajectoryType == CoordType.kAbsoluteResetPose) {
                // For AbsoluteResetPose trajectories, first command needs to be to reset the robot Pose
                addCommands(new DriveResetPose(trajectoryFacing.getInitialPose(), false, driveTrain, log));
            } else if (trajectoryType == CoordType.kAbsoluteResetPoseTol) {
                // For AbsoluteResetPoseTol trajectories, first command needs to be to reset the robot Pose
                addCommands(new DriveResetPose(trajectoryFacing.getInitialPose(), true, driveTrain, log));
            }

            swerveControllerLogCommand =
                new SwerveControllerLogCommand(
                    trajectoryFacing.trajectory,
                    driveTrain::getPose,
                    Constants.DriveConstants.kDriveKinematics,
                    new PIDController(Constants.TrajectoryConstants.kPXController, 0, 0),
                    new PIDController(Constants.TrajectoryConstants.kPYController, 0, 0),
                    thetaController,
                    () -> trajectoryFacing.finalRotation,
                    (a) -> driveTrain.setModuleStates(a, false),
                    log,
                    driveTrain);
        }

        // Next follow the trajectory
        addCommands(swerveControllerLogCommand);
        
        // Add any final commands, per the stopAtEnd
        if (stopAtEnd == StopType.kBrake) {
            addCommands(new DriveStop(driveTrain, log),
                new InstantCommand(() -> driveTrain.setDriveModeCoast(false))
            );
        } else if (stopAtEnd == StopType.kCoast) {
            addCommands(new DriveStop(driveTrain, log),
                new InstantCommand(() -> driveTrain.setDriveModeCoast(true))
            );
        }

         // Log that the command completed
        addCommands(new FileLogWrite(false, false, "DriveTrajectory", "Finish", log));
   }

   /**
     * Follows a trajectory with swerve drive.
     * @param trajectoryType Specify what robot starting position to use
     * kRelative = path starts where robot is, kAbsolute = path starts where it was told to regardless of whether the robot is actually there
     * kAbsoluteResetPose = path starts where it was told to and robot is set at that starting point/facing,
     * kAbsoluteResetPoseTol = path starts where it was told to and robot is set at that starting point/facing if it is not already close to that facing
     * @param stopAtEnd  True = robot stops at end of trajectory, False = robot does not end stopped
     * @param trajectoryFacings An array containing the trajectories to follow and the final facing of the robot, based on the alliance.
     * Note that the trajectoryFacing.initialRotation is *ignored*.
     * @param driveTrain The driveTrain subsystem to be controlled.
     * @param log        File for logging
     */
    public DriveTrajectory(CoordType trajectoryType, StopType stopAtEnd, TrajectoryFacingPair trajectoryFacings, DriveTrain driveTrain, AllianceSelection alliance, FileLog log) { 
        // Log that the command started
        addCommands(new FileLogWrite(false, false, "DriveTrajectory", "Start", log));

        // Define the controller for robot rotation
        ProfiledPIDController thetaController =
            new ProfiledPIDController(
                Constants.TrajectoryConstants.kPThetaController, 0, 0, Constants.TrajectoryConstants.kThetaControllerConstraints);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        // Create the serve controller command to follow the trajectory
        // Also add any initial commands before following the trajectory, depending on trajectoryType
        SwerveControllerLogCommand swerveControllerLogCommand;
        if (trajectoryType == CoordType.kRelative) {
            // For relative trajectories, first command needs to be to save the initial robot Pose
            addCommands(new InstantCommand(() -> initialPose = driveTrain.getPose()));

            swerveControllerLogCommand =
                new SwerveControllerLogCommand(
                    () -> ((alliance.getAlliance() == Alliance.Red) ? trajectoryFacings.red : trajectoryFacings.blue).trajectory,
                    // For relative trajectories, get the current pose relative to the initial robot Pose
                    () -> driveTrain.getPose().relativeTo(initialPose),  
                    Constants.DriveConstants.kDriveKinematics,
                    new PIDController(Constants.TrajectoryConstants.kPXController, 0, 0),
                    new PIDController(Constants.TrajectoryConstants.kPYController, 0, 0),
                    thetaController,
                    () -> ((alliance.getAlliance() == Alliance.Red) ? trajectoryFacings.red : trajectoryFacings.blue).finalRotation,
                    (a) -> driveTrain.setModuleStates(a, false),
                    log,
                    driveTrain);
        } else {
            if (trajectoryType == CoordType.kAbsoluteResetPose) {
                // For AbsoluteResetPose trajectories, first command needs to be to reset the robot Pose
                addCommands(new DriveResetPose(() -> ((alliance.getAlliance() == Alliance.Red) ? trajectoryFacings.red : trajectoryFacings.blue).getInitialPose(), false, driveTrain, log));
            } else if (trajectoryType == CoordType.kAbsoluteResetPoseTol) {
                // For AbsoluteResetPoseTol trajectories, first command needs to be to reset the robot Pose
                addCommands(new DriveResetPose(() -> ((alliance.getAlliance() == Alliance.Red) ? trajectoryFacings.red : trajectoryFacings.blue).getInitialPose(), true, driveTrain, log));
            }

            swerveControllerLogCommand =
                new SwerveControllerLogCommand(
                    () -> ((alliance.getAlliance() == Alliance.Red) ? trajectoryFacings.red : trajectoryFacings.blue).trajectory,
                    driveTrain::getPose,
                    Constants.DriveConstants.kDriveKinematics,
                    new PIDController(Constants.TrajectoryConstants.kPXController, 0, 0),
                    new PIDController(Constants.TrajectoryConstants.kPYController, 0, 0),
                    thetaController,
                    () -> ((alliance.getAlliance() == Alliance.Red) ? trajectoryFacings.red : trajectoryFacings.blue).finalRotation,
                    (a) -> driveTrain.setModuleStates(a, false),
                    log,
                    driveTrain);
        }

        // Next follow the trajectory
        addCommands(swerveControllerLogCommand);
        
        // Add any final commands, per the stopAtEnd
        if (stopAtEnd == StopType.kBrake) {
            addCommands(new DriveStop(driveTrain, log),
                new InstantCommand(() -> driveTrain.setDriveModeCoast(false))
            );
        } else if (stopAtEnd == StopType.kCoast) {
            addCommands(new DriveStop(driveTrain, log),
                new InstantCommand(() -> driveTrain.setDriveModeCoast(true))
            );
        }

         // Log that the command completed
        addCommands(new FileLogWrite(false, false, "DriveTrajectory", "Finish", log));
   }

   /**
     * Follows a choreo trajectory with swerve drive.
     * @param trajectoryType Specify what robot starting position to use
     * kRelative = path starts where robot is, kAbsolute = path starts where it was told to regardless of whether the robot is actually there
     * kAbsoluteResetPose = path starts where it was told to and robot is set at that starting point/facing,
     * kAbsoluteResetPoseTol = path starts where it was told to and robot is set at that starting point/facing if it is not already close to that facing
     * @param stopAtEnd  True = robot stops at end of trajectory, False = robot does not end stopped
     * @param trajectory A Swerve sample choreo trajectory that will be run
     * @param driveTrain The driveTrain subsystem to be controlled.
     * @param alliance   AllianceSelection class of robot
     * @param log        File for logging
     */
    public DriveTrajectory(CoordType trajectoryType, StopType stopAtEnd, Trajectory<SwerveSample> trajectory, DriveTrain driveTrain, AllianceSelection alliance, FileLog log) { 
        // Log that the command started
        addCommands(new FileLogWrite(false, false, "DriveTrajectory", "Start", log));
    
        // Define the controller for robot rotation
        PIDController thetaController =
            new PIDController(
                Constants.TrajectoryConstants.kPThetaController, 0, 0);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);
    
        // Create the serve controller command to follow the trajectory
        // Also add any initial commands before following the trajectory, depending on trajectoryType
        ChoreoFollower ChoreoFollower;
        if (trajectoryType == CoordType.kRelative) {
            // For relative trajectories, first command needs to be to save the initial robot Pose
            addCommands(new InstantCommand(() -> initialPose = driveTrain.getPose()));
        
            ChoreoFollower = 
                new ChoreoFollower(
                        trajectory, 
                        new PIDController(Constants.TrajectoryConstants.kPXController, 0.0, 0.0),
                        new PIDController(Constants.TrajectoryConstants.kPYController, 0.0, 0.0), 
                        thetaController, 
                        (speeds) -> driveTrain.drive(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond, speeds.omegaRadiansPerSecond, true, false), 
                        // For relative trajectories, get the current pose relative to the initial robot Pose
                        () -> driveTrain.getPose().relativeTo(initialPose), 
                        () -> alliance.getAlliance() == Alliance.Red, 
                        driveTrain, 
                        log);
        } else {
            if (trajectoryType == CoordType.kAbsoluteResetPose) {
                // For AbsoluteResetPose trajectories, first command needs to be to reset the robot Pose
                addCommands(
                    new DriveResetPose(() -> (trajectory.getInitialPose(alliance.getAlliance() == Alliance.Red).get()), false, driveTrain, log));
            } else if (trajectoryType == CoordType.kAbsoluteResetPoseTol) {
                // For AbsoluteResetPoseTol trajectories, first command needs to be to reset the robot Pose
                addCommands(
                    new DriveResetPose(() -> (trajectory.getInitialPose(alliance.getAlliance() == Alliance.Red).get()), true, driveTrain, log));
            }
        
            ChoreoFollower = 
                new ChoreoFollower(
                        trajectory, 
                        new PIDController(Constants.TrajectoryConstants.kPXController, 0.0, 0.0),
                        new PIDController(Constants.TrajectoryConstants.kPYController, 0.0, 0.0), 
                        thetaController, 
                        (speeds) -> driveTrain.drive(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond, speeds.omegaRadiansPerSecond, true, false), 
                        driveTrain::getPose,
                        () -> alliance.getAlliance() == Alliance.Red, 
                        driveTrain, 
                        log);
        }
    
        // Next follow the trajectory
        addCommands(ChoreoFollower);
        
        // Add any final commands, per the stopAtEnd
        if (stopAtEnd == StopType.kBrake) {
            addCommands(new DriveStop(driveTrain, log),
                new InstantCommand(() -> driveTrain.setDriveModeCoast(false))
            );
        } else if (stopAtEnd == StopType.kCoast) {
            addCommands(new DriveStop(driveTrain, log),
                new InstantCommand(() -> driveTrain.setDriveModeCoast(true))
            );
        }
    
         // Log that the command completed
        addCommands(new FileLogWrite(false, false, "DriveTrajectory", "Finish", log));
    }

}
