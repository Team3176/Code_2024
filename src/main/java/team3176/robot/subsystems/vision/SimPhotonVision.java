package team3176.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.List;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import team3176.robot.subsystems.drivetrain.Drivetrain;

public class SimPhotonVision extends SubsystemBase {
  // Simulated Vision System.

  VisionSystemSim simVision = new VisionSystemSim("photonvision");
  PhotonPoseEstimator estimator;

  // public SimPhotonVision(List<PhotonCamera> c, List<Transform3d> t, AprilTagFieldLayout field) {
  //   for (int i = 0; i < c.size(); i++) {
  //     Transform3d camera2Robot = t.get(i);
  //     PhotonCameraSim simCam =
  //         new PhotonCameraSim(c.get(i), arducam_720(), 0.07, Units.feetToMeters(30));
  //     simVision.addCamera(simCam, camera2Robot);
  //   }

  //   // simVision.addVisionTargets(new VisionTargetSim(t2pose,TargetModel.kTag16h5,2));
  //   simVision.addAprilTags(field);
  // }
  public SimPhotonVision(List<LoggedAprilPhotonCam> l, AprilTagFieldLayout field) {
    for (LoggedAprilPhotonCam c : l) {
      PhotonCameraSim simCam = new PhotonCameraSim(c.getCamera(), arducam_720(), 0.07, 6.0);
      simVision.addCamera(simCam, c.getRobot2Camera());
    }
    simVision.addAprilTags(field);
  }

  public void switchAllaince(AprilTagFieldLayout field) {
    simVision.removeVisionTargets("apriltags");
    simVision.addAprilTags(field);
  }

  @Override
  public void periodic() {

    Pose2d currentPose = Drivetrain.getInstance().getSimNoNoisePose();
    simVision.update(currentPose);
  }

  public static SimCameraProperties arducam_720() {
    var prop = new SimCameraProperties();
    prop.setCalibration(
        1280,
        720,
        Matrix.mat(Nat.N3(), Nat.N3())
            .fill( // intrinsic
                879.0720598169321,
                0,
                604.9212300278572,
                0,
                879.1507505285007,
                389.16742755726875,
                0.0,
                0.0,
                1.0),
        VecBuilder.fill( // distort
            -0.05910651726620547,
            0.04721812350044875,
            0.0023418721710541947,
            0.00018013724744152414,
            0.1608556267669461));
    prop.setCalibError(0.37, 0.06);
    prop.setFPS(35);
    prop.setAvgLatencyMs(25);
    prop.setLatencyStdDevMs(10);
    return prop;
  }
}
