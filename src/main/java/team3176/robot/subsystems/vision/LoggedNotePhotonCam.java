package team3176.robot.subsystems.vision;

import org.littletonrobotics.junction.Logger;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class LoggedNotePhotonCam {

  String name = "notecam";

  public double noteYaw;
  public double notePitch;
  public boolean seeNote;

  private PhotonCameraIO io;

  private PhotonCameraInputsAutoLogged inputs;

  public LoggedNotePhotonCam() {
    this.io = new PhotonCameraIO(name);
    this.inputs = new PhotonCameraInputsAutoLogged();
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("photonvision/notecam", inputs);
    PhotonPipelineResult results = io.getResult(inputs.rawBytes);
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
