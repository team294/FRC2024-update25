package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstants.PhotonVisionConstants;
import frc.robot.utilities.FileLog;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class NotePhotonCameraWrapper extends SubsystemBase {
  public PhotonCamera photonCamera;
  private FileLog log;
  private boolean hasInit = false;
  private int logRotationKey;
  private boolean fastLogging = false;

  public NotePhotonCameraWrapper(FileLog log, int logRotationKey) {
    this.log = log;
    this.logRotationKey = logRotationKey;
  }

  /**
   * Turns file logging on every scheduler cycle (~20ms) or every 10 cycles (~0.2 sec)
   * @param enabled true = every cycle, false = every 10 cycles
   */ 
  public void enableFastLogging(boolean enabled) {
    this.fastLogging = enabled;
  }

  public void init() {
    log.writeLog(true, "NotePhotonCameraWrapper", "Init", "Starting");

    if (photonCamera == null) {
      photonCamera = new PhotonCamera(PhotonVisionConstants.noteCameraName);
    }
      
    hasInit = true;

    log.writeLog(true, "NotePhotonCameraWrapper", "Init", "Done");
  }

  public boolean hasInit() {
    return hasInit;
  }

  public void periodic() {
    // if (fastLogging || log.isMyLogRotation(logRotationKey)) {
    //   log.writeLog(false, "NotePhotonCameraWrapper", "Periodic", "");
    // }
  }

  /**
     * Returns the best target in this pipeline result. If there are no targets, this method will
     * return null. The best target is determined by the target sort mode in the PhotonVision UI.
     *
     * @return The best target of the pipeline result.
     */
  public PhotonTrackedTarget getBestTarget() {
    return photonCamera.getLatestResult().getBestTarget();
  }

  public PhotonPipelineResult getLatestResult() {
    return photonCamera.getLatestResult();
  }
}