package team3176.robot.subsystems.vision;

import java.util.ArrayList;
import org.littletonrobotics.junction.Logger;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.TargetCorner;
import team3176.robot.subsystems.leds.LEDS;

public class LoggedNotePhotonCam {

  String name = "notecam";

  public double noteYaw;
  public double notePitch;
  public boolean seeNote;
  public boolean lastSeeNote;

  private LEDS leds;
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
      if (seeNote != lastSeeNote) {
        leds.getInstance().setWantNote(true);
        lastSeeNote = seeNote;
      }
      ArrayList<Double> x = new ArrayList<Double>();
      ArrayList<Double> y = new ArrayList<Double>();
      for (PhotonTrackedTarget t : results.getTargets()) {
        for (TargetCorner c : t.getMinAreaRectCorners()) {
          x.add(c.x);
          y.add(c.y);
        }
      }
      double[] xarray = x.stream().mapToDouble(Double::doubleValue).toArray();
      double[] yarray = y.stream().mapToDouble(Double::doubleValue).toArray();
      Logger.recordOutput("photonvision/" + name + "/xarray", xarray);
      Logger.recordOutput("photonvision/" + name + "/yarray", yarray);
    } else {
      seeNote = false;
      if (seeNote != lastSeeNote) {
        leds.getInstance().setWantNote(false);
        lastSeeNote = seeNote;
      }
      ArrayList<Double> x = new ArrayList<Double>();
      ArrayList<Double> y = new ArrayList<Double>();
      for (PhotonTrackedTarget t : results.getTargets()) {
        for (TargetCorner c : t.getMinAreaRectCorners()) {
          x.add(c.x);
          y.add(c.y);
        }
      }
      Logger.recordOutput("photonvision/" + name + "/xarray", new double[] {});
      Logger.recordOutput("photonvision/" + name + "/yarray", new double[] {});
    }
  }
}
