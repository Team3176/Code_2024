package team3176.robot.subsystems.vision;

import java.util.ArrayList;
import org.littletonrobotics.junction.Logger;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.TargetCorner;

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

  public void logExtra(PhotonPipelineResult results) {
    ArrayList<Double> x = new ArrayList<Double>();
    ArrayList<Double> y = new ArrayList<Double>();
    if (results.hasTargets()) {

      for (PhotonTrackedTarget t : results.getTargets()) {
        for (TargetCorner c : t.getMinAreaRectCorners()) {
          x.add(c.x);
          y.add(c.y);
        }
      }

    } else {

    }
    double[] xarray = x.stream().mapToDouble(Double::doubleValue).toArray();
    double[] yarray = y.stream().mapToDouble(Double::doubleValue).toArray();
    Logger.recordOutput("photonvision/" + name + "/xarray", xarray);
    Logger.recordOutput("photonvision/" + name + "/yarray", yarray);
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("photonvision/" + this.name, inputs);
    var results = inputs.results;
    logExtra(results);
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
