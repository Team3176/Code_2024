package team3176.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.ArrayList;
import java.util.List;
import org.littletonrobotics.junction.Logger;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import team3176.robot.Constants;
import team3176.robot.Constants.Mode;

public class PhotonVisionSystem extends SubsystemBase {
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
  public static final Transform3d Robot2camera1 =
      new Transform3d(
          new Translation3d(
              Units.inchesToMeters(13), Units.inchesToMeters(-10), Units.inchesToMeters(8)),
          new Rotation3d(
              Units.degreesToRadians(0), Units.degreesToRadians(-25), Units.degreesToRadians(0)));
  public static final Transform3d Robot2camera2 =
      new Transform3d(
          new Translation3d(
              Units.inchesToMeters(25 / 2 - 1),
              Units.inchesToMeters(-25 / 2 + 1),
              Units.inchesToMeters(10)),
          new Rotation3d(
              Units.degreesToRadians(0), Units.degreesToRadians(-10), Units.degreesToRadians(20)));
  public static final Transform3d Robot2camera3 =
      new Transform3d(
          new Translation3d(
              Units.inchesToMeters(-25 / 2 + 1),
              Units.inchesToMeters(25 / 2 - 1),
              Units.inchesToMeters(10)),
          new Rotation3d(
              Units.degreesToRadians(0),
              Units.degreesToRadians(-10),
              Units.degreesToRadians(-180 + 20)));
  public static final Transform3d Robot2camera4 =
      new Transform3d(
          new Translation3d(
              Units.inchesToMeters(-25 / 2 + 1),
              Units.inchesToMeters(-25 / 2 + 1),
              Units.inchesToMeters(10)),
          new Rotation3d(
              Units.degreesToRadians(0),
              Units.degreesToRadians(-10),
              Units.degreesToRadians(-180 - 20)));
  private ArrayList<LoggedPhotonCam> cameras = new ArrayList<LoggedPhotonCam>();

  public double noteYaw;
  public double notePitch;
  public boolean seeNote;

  public Integer camera1tagSeen;
  public double camera1yaw;
  public double camera1pitch;
  public boolean camera1seetag;
  public Transform2d camer1tagposition2d;
  public Transform3d camera1tagposition3d;

  List<Pose3d> visionTargets = new ArrayList<>();

  AprilTagFieldLayout field;
  private static PhotonVisionSystem instance;
  private SimPhotonVision simInstance;
  EstimatedRobotPose currentEstimate;
  PhotonCamera notecamera;
  PhotonCamera camera1;

  private PhotonVisionSystem() {
    // cameras.add(new LoggedPhotonCam("camera1", Robot2camera1));
    notecamera = new PhotonCamera("notecam");
    camera1 = new PhotonCamera("camera1");
    // cameras.add(new LoggedPhotonCam("camera2", Robot2camera2));
    // cameras.add(new LoggedPhotonCam("camera3", Robot2camera3));
    // cameras.add(new LoggedPhotonCam("camera4", Robot2camera4));
    try {
      field = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
      field.setOrigin(OriginPosition.kBlueAllianceWallRightSide);
    } catch (Exception e) {
      System.out.println("woops can't load the field");
    }
    if (Constants.getMode() == Mode.SIM) {
      simInstance = new SimPhotonVision(cameras, field);
    }
  }

  public static PhotonVisionSystem getInstance() {
    if (instance == null) {
      instance = new PhotonVisionSystem();
    }
    return instance;
  }

  @Override
  public void periodic() {
    visionTargets.clear();
    // Drivetrain.getInstance().visionPose3d = cameras.get(0).getPoseEstimates().get(0)
    for (LoggedPhotonCam c : cameras) {
      c.periodic();
      visionTargets.addAll(c.getCamera2Target());
    }
    Logger.recordOutput(
        "photonvision/visionTargets", visionTargets.toArray(new Pose3d[visionTargets.size()]));

    PhotonPipelineResult result = notecamera.getLatestResult();
    Logger.recordOutput("photonvision/noteraw", result);
    if (result.hasTargets()) {
      PhotonTrackedTarget target = result.getBestTarget();
      Logger.recordOutput("photonvision/noteyaw", target.getYaw());
      Logger.recordOutput("photonvision/notepitch", target.getPitch());
      noteYaw = target.getYaw();
      notePitch = target.getPitch();
      double noteskew = target.getSkew();
      double notearea = target.getArea();

    } else {
      seeNote = false;
    }

    PhotonPipelineResult result2 = camera1.getLatestResult();
    if (result2.hasTargets()) {
      PhotonTrackedTarget target2 = result2.getBestTarget();
      camera1pitch = target2.getPitch();
      camera1yaw = target2.getYaw();
      camera1tagSeen = target2.getFiducialId();
      camera1tagposition3d = target2.getBestCameraToTarget();
      // camer1tagposition2d = target2.getCameraToTarget();
    } else {
      camera1seetag = false;
    }
  }
}
