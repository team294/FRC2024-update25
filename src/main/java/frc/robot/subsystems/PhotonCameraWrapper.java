package frc.robot.subsystems;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstants.PhotonVisionConstants;
import frc.robot.utilities.AllianceSelection;
import frc.robot.utilities.FileLog;

import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

// import javax.lang.model.util.Elements.Origin;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;

public class PhotonCameraWrapper extends SubsystemBase {
  public PhotonCamera photonCamera;
  public PhotonCamera photonCamera2;
  public PhotonPoseEstimator photonPoseEstimatorCamera1;
  public PhotonPoseEstimator photonPoseEstimatorCamera2;
  private AprilTagFieldLayout aprilTagFieldLayout;
  private FileLog log;
  private boolean hasInit = false;
  private Alliance currAlliance = Alliance.Blue;
  private AllianceSelection allianceSelection;
  private int logRotationKey;
  private boolean fastLogging = false;
  
  public enum Cameras{
    backCamera(PhotonVisionConstants.aprilTagCameraName),
    frontCamera(PhotonVisionConstants.aprilTagCameraBackName);

    @SuppressWarnings({"MemberName", "PMD.SingularField"})
    public final String cameraName;
    Cameras(String cameraName){ this.cameraName = cameraName; }
  }

  public PhotonCameraWrapper(AllianceSelection allianceSelection, FileLog log, int logRotationKey) {
    this.log = log;
    this.logRotationKey = logRotationKey;
    this.allianceSelection = allianceSelection;
  }

  /**
   * Turns file logging on every scheduler cycle (~20ms) or every 10 cycles (~0.2 sec)
   * @param enabled true = every cycle, false = every 10 cycles
   */ 
  public void enableFastLogging(boolean enabled) {
    this.fastLogging = enabled;
  }

  public void init() {
    log.writeLog(true, "PhotonCameraWrapper", "Init", "Starting");

    currAlliance = allianceSelection.getAlliance();

    if (photonCamera == null) {
      photonCamera = new PhotonCamera(PhotonVisionConstants.aprilTagCameraName);
    }

    if (photonCamera2 == null){
      photonCamera2 = new PhotonCamera(PhotonVisionConstants.aprilTagCameraBackName);
    }

    //  aprilTagFieldLayout = field.getAprilTagFieldLayout();

    try {
      aprilTagFieldLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2025Reefscape.m_resourceFile);
      currAlliance = allianceSelection.getAlliance();
      switch (currAlliance) {
        case Blue:
          aprilTagFieldLayout.setOrigin(OriginPosition.kBlueAllianceWallRightSide);         // note: we can remove the logic for the red side if we are going to do blue only odometry
          break;
        case Red:
          aprilTagFieldLayout.setOrigin(OriginPosition.kRedAllianceWallRightSide);
          break;
        default:
          log.writeLog(true, "PhotonCameraWrapper", "UpdateAlliance", "Alliance invalid");
          break;
      }
      log.writeLog(true, "PhotonCameraWrapper", "Init", "Loaded april tags from file");
    } catch (IOException e) {
      log.writeLog(true, "PhotonCameraWrapper", "Init", "Error loading april tags from file");
      e.printStackTrace();
    }
    
    // Create pose estimator
    photonPoseEstimatorCamera1 = new PhotonPoseEstimator(
      aprilTagFieldLayout,
      PoseStrategy.CLOSEST_TO_REFERENCE_POSE,
      PhotonVisionConstants.robotToCamBack);

    photonPoseEstimatorCamera2 = new PhotonPoseEstimator(
      aprilTagFieldLayout,
      PoseStrategy.CLOSEST_TO_REFERENCE_POSE,
      PhotonVisionConstants.robotToCamFront);
      
    hasInit = true;

    log.writeLog(true, "PhotonCameraWrapper", "Init", "Done");
  }

  public boolean hasInit() {
    return hasInit;
  }

  public Alliance getAlliance() {
    return allianceSelection.getAlliance();
  }

  public void periodic() {
    if (allianceSelection.getAlliance() != currAlliance) {
      init();
      log.writeLogEcho(true, "PhotonCameraWrapper", "UpdateAlliance", "Alliance changed", currAlliance);
    }

    if (fastLogging || log.isMyLogRotation(logRotationKey)) {
      // log.writeLog(false, "PhotonCameraWrapper", "Periodic", "");
    }
  }

 

  /**
   * Returns a list of the best targets in the camera pipelines with its corresponding cameras. If there is no new targets, the cameras element will return null
   * 
   * @return A list of the cameras best targets
   */
  List<Pair<PhotonPipelineResult, Cameras>> getLatestCameraResults(){
    List<Pair<PhotonPipelineResult, Cameras>> results = new ArrayList<Pair<PhotonPipelineResult, Cameras>>();
    results.add(getLatestResultCamera1());
    results.add(getLatestResultCamera2());
    return results;
  }

   /**
     * Returns a pair with the best target in Camera 1 Pipeline result and the Camera enum. If there are no new targets, this method will
     * return null. The best target is determined by the target sort mode in the PhotonVision UI.
     *
     * @return The best target of the pipeline result.
     */
  Pair<PhotonPipelineResult, Cameras> getLatestResultCamera1() {
    List<PhotonPipelineResult> results = photonCamera.getAllUnreadResults();
    return results.isEmpty() ? null : new Pair<PhotonPipelineResult, Cameras> (results.get(results.size()-1), Cameras.backCamera);
  }

   /**
     * Returns a pair with the best target in Camera 2 Pipeline result and the Camera enum. If there are no new targets, this method will
     * return null. The best target is determined by the target sort mode in the PhotonVision UI.
     *
     * @return The best target of the pipeline result.
     */
  Pair<PhotonPipelineResult, Cameras> getLatestResultCamera2(){
    List<PhotonPipelineResult> results = photonCamera2.getAllUnreadResults();
    return results.isEmpty() ? null : new Pair<PhotonPipelineResult, Cameras>(results.get(results.size() - 1), Cameras.frontCamera);
  }

  /**
  * @param estimatedRobotPose The current best guess at robot pose
  * @return A pair of the fused camera observations to a single Pose2d on the
  *         field, and the time
  *         of the observation. Assumes a planar field and the robot is always
  *         firmly on the ground
  */
  public Optional<EstimatedRobotPose> getEstimatedGlobalPose(Pose2d prevEstimatedRobotPose, PhotonPipelineResult latestResult, Cameras cameraName) {
    Optional<EstimatedRobotPose> newPoseOptional;
    switch(cameraName){
      case backCamera:
        photonPoseEstimatorCamera1.setReferencePose(prevEstimatedRobotPose);
        newPoseOptional = photonPoseEstimatorCamera1.update(latestResult);
        if (newPoseOptional.isPresent()) {
          EstimatedRobotPose newPose = newPoseOptional.get();
          if(fastLogging || log.isMyLogRotation(logRotationKey)) {
            log.writeLog(false, "PhotonCameraWrapper", "getEstimatedGlobalPose", "IsConnected", photonCamera.isConnected(), "TagPresent", true, "X",newPose.estimatedPose.getX(),"Y",newPose.estimatedPose.getY());
            SmartDashboard.putBoolean("PhotonVision Connected", photonCamera.isConnected());
          }
        } else {
          if(fastLogging || log.isMyLogRotation(logRotationKey)) {
            log.writeLog(false, "PhotonCameraWrapper", "getEstimatedGlobalPose", "IsConnected", photonCamera.isConnected(), "TagPresent", false, "X", 0, "Y", 0);
            SmartDashboard.putBoolean("PhotonVision Connected", photonCamera.isConnected());
          }
        }
      break;
      case frontCamera:
        photonPoseEstimatorCamera2.setReferencePose(prevEstimatedRobotPose);
        newPoseOptional = photonPoseEstimatorCamera1.update(latestResult);
        if (newPoseOptional.isPresent()) {
          EstimatedRobotPose newPose = newPoseOptional.get();
          if(fastLogging || log.isMyLogRotation(logRotationKey)) {
            log.writeLog(false, "PhotonCameraWrapper", "getEstimatedGlobalPose", "IsConnected", photonCamera.isConnected(), "TagPresent", true, "X",newPose.estimatedPose.getX(),"Y",newPose.estimatedPose.getY());
            SmartDashboard.putBoolean("PhotonVision Connected", photonCamera.isConnected());
          }
        } else {
          if(fastLogging || log.isMyLogRotation(logRotationKey)) {
            log.writeLog(false, "PhotonCameraWrapper", "getEstimatedGlobalPose", "IsConnected", photonCamera.isConnected(), "TagPresent", false, "X", 0, "Y", 0);
            SmartDashboard.putBoolean("PhotonVision Connected", photonCamera.isConnected());
          }
        }
      break;
      default:
        newPoseOptional = Optional.empty();
    }
    return newPoseOptional;
  }
}