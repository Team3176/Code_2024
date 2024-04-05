package team3176.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import org.littletonrobotics.junction.Logger;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.TargetCorner;
import team3176.robot.Constants;
import team3176.robot.Constants.Mode;
import team3176.robot.subsystems.drivetrain.Drivetrain;
import team3176.robot.util.TunableTransform3d;

public class LoggedAprilPhotonCam {
  // currently using an body frame that is at the center of the XY of the robot and projected down
  // to the floor Z
  // is an area for further standards with design to pick a body attached frame
  /*
   *     ^ +x
   *     |
   * +y  |
   * <---O +z up
   *
   *
   * cameras have the convention similar reporting position in the camera frame with +x being the axis through the lense and +y to the left, z up
   */
  private Transform3d robot2Camera =
      new Transform3d(
          new Translation3d(
              Units.inchesToMeters(25 / 2 - 1),
              Units.inchesToMeters(25 / 2 - 1),
              Units.inchesToMeters(10)),
          new Rotation3d(
              Units.degreesToRadians(0), Units.degreesToRadians(-10), Units.degreesToRadians(-20)));
  private TunableTransform3d robot2CameraTune;
  private PhotonCameraIO io;
  private PhotonCameraInputsAutoLogged inputs;
  // private LoggedPhotonCamera cam;
  private List<Pose3d> targets = new ArrayList<Pose3d>();
  private List<Pose3d> estimates = new ArrayList<Pose3d>();
  String name = "";
  AprilTagFieldLayout field;
  PhotonPoseEstimator estimator;
  EstimatedRobotPose currentEstimate;

  public LoggedAprilPhotonCam(String name, Transform3d robot2Camera) {
    this.name = name;
    // this.cam = new LoggedPhotonCamera(name);
    this.io = new PhotonCameraIO(name);
    this.inputs = new PhotonCameraInputsAutoLogged();
    this.robot2Camera = robot2Camera;
    this.robot2CameraTune =
        new TunableTransform3d(
            name,
            robot2Camera.getX(),
            robot2Camera.getY(),
            robot2Camera.getZ(),
            robot2Camera.getRotation().getX(),
            robot2Camera.getRotation().getX(),
            robot2Camera.getRotation().getX());
    try {
      field = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
      field.setOrigin(OriginPosition.kBlueAllianceWallRightSide);
    } catch (Exception e) {
      System.out.println("woops can't load the field");
    }
    estimator =
        new PhotonPoseEstimator(
            field, PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, robot2Camera);
    estimator.setMultiTagFallbackStrategy(PoseStrategy.AVERAGE_BEST_TARGETS);
  }

  public Transform3d getRobot2Camera() {
    return robot2Camera;
  }

  public PhotonCamera getCamera() {
    return io.getCamera();
  }

  public void generateLoggingData(PhotonPipelineResult results) {
    targets.clear();
    robot2CameraTune.checkParemeterUpdate();

    if (results.hasTargets()) {
      Pose3d current3d = new Pose3d(Drivetrain.getInstance().getPose());

      estimates.clear();
      ArrayList<Double> x = new ArrayList<Double>();
      ArrayList<Double> y = new ArrayList<Double>();
      Logger.recordOutput("photonvision/" + name + "/num targets", results.getTargets().size());
      for (PhotonTrackedTarget t : results.getTargets()) {
        targets.add(current3d.transformBy(robot2Camera).transformBy(t.getBestCameraToTarget()));
        for (TargetCorner c : t.getDetectedCorners()) {
          x.add(c.x);
          y.add(c.y);
        }

        if (field.getTagPose(t.getFiducialId()).isPresent()) {
          estimates.add(
              PhotonUtils.estimateFieldToRobotAprilTag(
                  t.getBestCameraToTarget(),
                  field.getTagPose(t.getFiducialId()).get(),
                  robot2Camera.inverse()));
        }
      }
      double[] xarray = x.stream().mapToDouble(Double::doubleValue).toArray();
      double[] yarray = y.stream().mapToDouble(Double::doubleValue).toArray();
      Logger.recordOutput("photonvision/" + name + "/xarray", xarray);
      Logger.recordOutput("photonvision/" + name + "/yarray", yarray);
      Logger.recordOutput(
          "photonvision/" + name + "/targetposes", targets.toArray(new Pose3d[targets.size()]));
      Logger.recordOutput(
          "photonvision/" + name + "/poseEstimates",
          estimates.toArray(new Pose3d[estimates.size()]));
    } else {
      Logger.recordOutput("photonvision/" + name + "/num targets", results.getTargets().size());
      Logger.recordOutput("photonvision/" + name + "/targetposes", new Pose3d[] {});
      Logger.recordOutput("photonvision/" + name + "/poseEstimates", new Pose3d[] {});
      double[] zeros = {0};
      Logger.recordOutput("photonvision/" + name + "/xarray", zeros);
      Logger.recordOutput("photonvision/" + name + "/yarray", zeros);
    }
  }

  public List<Pose3d> getCamera2Target() {
    return targets;
  }

  public List<Pose3d> getPoseEstimates() {
    return estimates;
  }

  public void LogCameraPose() {
    Logger.recordOutput(
        "photonvision/" + name + "/cameraPose",
        new Pose3d(Drivetrain.getInstance().getPose()).transformBy(robot2Camera));
  }

  public void filterAndAddVisionPose(EstimatedRobotPose p) {
    Matrix<N3, N1> cov = new Matrix<>(Nat.N3(), Nat.N1());
    double distance = 0.0;
    for (var t : p.targetsUsed) {
      distance += t.getBestCameraToTarget().getTranslation().getNorm() / p.targetsUsed.size();
    }
    Logger.recordOutput("photonvision/" + name + "/distance2target", distance);
    if (p.targetsUsed.size() > 1) {
      // multi tag
      double distance2 = Math.max(Math.pow(distance * 0.4, 2), 0.7);
      cov = VecBuilder.fill(distance2, distance2, 0.9);
    } else {
      double distance2 = Math.pow(distance * 1.2, 2);
      cov = VecBuilder.fill(distance2, distance2, 100);
    }
    if (!DriverStation.isDisabled()) {
      if (p.targetsUsed.size() == 1) {
        if (Math.abs(p.estimatedPose.getZ()) > 1.0
            || p.estimatedPose
                    .minus(new Pose3d(Drivetrain.getInstance().getPose()))
                    .getTranslation()
                    .getNorm()
                > 1.0
            || distance > 7.0) {
          return;
        }
      }
    }
    // TODO: Remember to fix me for camera2
    if (!name.equals("camera2")) {
      Drivetrain.getInstance().addVisionMeasurement(p.estimatedPose, p.timestampSeconds, cov);
    }
  }

  public void periodic() {
    LogCameraPose();
    io.updateInputs(inputs);
    Logger.processInputs("photonvision/" + this.name, inputs);
    PhotonPipelineResult results = io.getResult(inputs.rawBytes);
    results.setTimestampSeconds(inputs.timestamp);
    if (Constants.getMode() == Mode.REPLAY) {
      Logger.recordOutput("photonvision/" + name + "/raw", PhotonPipelineResult.proto, results);
    }
    generateLoggingData(results);
    // Logger.recordOutput("photonvision/" + name + "/timestamp", results.getTimestampSeconds());
    Optional<EstimatedRobotPose> poseEst = estimator.update(results);
    if (poseEst.isPresent()) {
      filterAndAddVisionPose(poseEst.get());
      Logger.recordOutput("photonvision/" + name + "/bestPose", poseEst.get().estimatedPose);
    } else {
      Logger.recordOutput("photonvision/" + name + "/bestPose", new Pose3d[] {});
    }
  }
}
