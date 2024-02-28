package team3176.robot.subsystems.vision;

import org.littletonrobotics.junction.Logger;
import org.photonvision.targeting.PhotonTrackedTarget;

public class LoggedNotePhotonCam {

  private PhotonCameraIO io;
  private PhotonCameraInputsAutoLogged inputs;
  String name = "notecam";
  public double noteYaw;
  public double notePitch;
  public boolean seeNote;

  public LoggedNotePhotonCam() {
    this.io = new PhotonCameraIO(name);
    this.inputs = new PhotonCameraInputsAutoLogged();
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("photonvision/" + this.name, inputs);
    var results = inputs.results;
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
