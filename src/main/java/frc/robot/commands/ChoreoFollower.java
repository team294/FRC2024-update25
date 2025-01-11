// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.Optional;
import java.util.function.BooleanSupplier;
import java.util.function.Consumer;
import java.util.function.Supplier;

import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveTrain;
import frc.robot.utilities.FileLog;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ChoreoFollower extends Command {
  /** Creates a new ChoreoFollower. */
  private Timer timer;
  private Trajectory<SwerveSample> trajectory;
  private PIDController xController;
  private PIDController yController;
  private PIDController rotationController;
  private Consumer<ChassisSpeeds> outputChassisSpeed;
  private Supplier<Pose2d> poseSupplier;
  private BooleanSupplier mirrorTrajectory;
  private DriveTrain driveTrain;
  private FileLog log;



  /**
   * Choreo follower used to follow Choreo trajectories. 
   * This method takes in a choreo trajectory and follows the contents of that trajectory until the
   * the trajectory is finished 
   * 
   * @param trajectory Choreo trajectory to follow
   * @param xController Trajectory PID X feedback controller
   * @param yController  Trajectory PID Y feedback controller 
   * @param rotationController Roation PID feedback controller
   * @param outputChassisSpeeds a function that takes in a chassis speeds and controls the robot
   * @param poseSupplier a pose supplier to get the current robot position
   * @param mirrorTrajectory  determines if the trajectory will be mirrored: when true the trajectory will mirror while internal robot odometry will remain the same.
   *                            When false the trajectory will not be changed
   * @param driveTrain  Drive Train subsystem
   * @param log LogFile for logging
   */
  public ChoreoFollower(Trajectory<SwerveSample> trajectory, PIDController xController, PIDController yController, PIDController rotationController, Consumer<ChassisSpeeds> outputChassisSpeeds, Supplier<Pose2d> poseSupplier, BooleanSupplier mirrorTrajectory, DriveTrain driveTrain, FileLog log) {
    this.trajectory = trajectory;
    this.xController = xController;
    this.yController = yController;
    this.rotationController = rotationController;
    this.outputChassisSpeed = outputChassisSpeeds;
    this.poseSupplier = poseSupplier;
    this.mirrorTrajectory = mirrorTrajectory;
    this.driveTrain = driveTrain;
    this.log = log;
    addRequirements(driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    this.timer = new Timer();
    timer.restart();
    log.writeLog(false, "ChoreoCommandFollower", "Start", 
    "Is Flipped", mirrorTrajectory.getAsBoolean());
    rotationController.enableContinuousInput(-Math.PI, Math.PI);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Optional<SwerveSample> sample = trajectory.sampleAt(timer.get(), mirrorTrajectory.getAsBoolean());
    double xFF = sample.get().vx;
    double yFF = sample.get().vy;
    double rotationFF = sample.get().omega;

    double xFeedback ;
    double yFeedback;
    double rotationFeedback;
    if(!sample.isEmpty()){
      xFeedback = xController.calculate(poseSupplier.get().getX(), sample.get().x);
      yFeedback = yController.calculate(poseSupplier.get().getY(), sample.get().y);
      rotationFeedback = 
          rotationController.calculate(poseSupplier.get().getRotation().getRadians(), sample.get().heading);
        
        log.writeLog(false, "choreoTrajectoryFollowerCommand", "State", 
          "Time", sample.get().t,
          "Traj X", sample.get().x,
          "Traj Y", sample.get().y,
          "Traj Vel", Math.hypot(sample.get().vx, sample.get().vy),
          "Traj VelAng", sample.get().omega,
          "Target rot", sample.get().heading,
          "Robot X", poseSupplier.get().getTranslation().getX(),
          "Robot Y", poseSupplier.get().getTranslation().getY(),
          "Robot Vel", Math.hypot(driveTrain.getRobotSpeeds().vxMetersPerSecond, driveTrain.getRobotSpeeds().vyMetersPerSecond),
          "Robot VelAng", Math.toDegrees(Math.atan2(driveTrain.getRobotSpeeds().vyMetersPerSecond, driveTrain.getRobotSpeeds().vxMetersPerSecond)),
          "Robot rot", driveTrain.getPose().getRotation().getDegrees());
        
    } 
    else{
      xFF = 0;
      yFF = 0;
      rotationFF = 0;
      xFeedback = xController.calculate(poseSupplier.get().getX(), trajectory.getFinalSample(mirrorTrajectory.getAsBoolean()).get().x);
      yFeedback = yController.calculate(poseSupplier.get().getY(), trajectory.getFinalSample(mirrorTrajectory.getAsBoolean()).get().y);
      rotationFeedback = 
        rotationController.calculate(poseSupplier.get().getRotation().getRadians(), trajectory.getFinalSample(mirrorTrajectory.getAsBoolean()).get().heading);
    }
    
    outputChassisSpeed.accept(new ChassisSpeeds(xFF + xFeedback, yFF + yFeedback, rotationFF + rotationFeedback)); 
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    timer.stop();
    outputChassisSpeed.accept(new ChassisSpeeds());  
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    Optional<Pose2d> finalPose = mirrorTrajectory.getAsBoolean() ? 
      trajectory.getFinalPose(true) : 
      trajectory.getFinalPose(false);
      if(finalPose.isEmpty()){
        log.writeLogEcho(true, "choreoTrajectoryFollowerCommand", "No final Pose exists");
        return true;
      }
    return timer.hasElapsed(trajectory.getTotalTime())
      && Math.abs(finalPose.get().getX() - poseSupplier.get().getX()) <= 0.5 
      && Math.abs(finalPose.get().getY() - poseSupplier.get().getY()) <= 0.5 
      && Math.abs(poseSupplier.get().getRotation().getDegrees() - finalPose.get().getRotation().getDegrees()) <= 20 
      && Math.hypot(driveTrain.getChassisSpeeds().vxMetersPerSecond, driveTrain.getChassisSpeeds().vyMetersPerSecond) <= 0.75;
  }
}
