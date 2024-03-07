package team3176.robot.subsystems.vision;

import org.littletonrobotics.junction.Logger;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import team3176.robot.util.vision.LoggedPhotonCamera;

public class LoggedNotePhotonCam {

  String name = "notecam";
  LoggedPhotonCamera cam;
  public double noteYaw;
  public double notePitch;
  public boolean seeNote;

  public LoggedNotePhotonCam() {
    cam = new LoggedPhotonCamera(name);
  }

  public void periodic() {
    PhotonPipelineResult results;
    if (cam.isConnected()) {
      results = cam.getLatestResult();
    } else {
      results = new PhotonPipelineResult();
    }
    if (results.hasTargets()) {
      PhotonTrackedTarget target = results.getBestTarget();
      Logger.recordOutput("photonvision/noteyaw", target.getYaw());
      Logger.recordOutput("photonvision/notepitch", target.getPitch());
      noteYaw = target.getYaw();
      notePitch = target.getPitch();
      double noteskew = target.getSkew();
      double notearea = target.getArea();
      seeNote = true;
    } else {
      seeNote = false;
    }
  }
}
