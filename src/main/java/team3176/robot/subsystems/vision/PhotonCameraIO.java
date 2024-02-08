package team3176.robot.subsystems.vision;

import org.littletonrobotics.junction.AutoLog;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;

public class PhotonCameraIO {
  private PhotonCamera cam;

  @AutoLog
  public static class PhotonCameraInputs {
    PhotonPipelineResult results = new PhotonPipelineResult();
  }

  public PhotonCameraIO(String name) {
    this.cam = new PhotonCamera(name);
  }

  public void updateInputs(PhotonCameraInputs inputs) {
    inputs.results = this.cam.getLatestResult();
  }

  public PhotonCamera getCamera() {
    return this.cam;
  }
}