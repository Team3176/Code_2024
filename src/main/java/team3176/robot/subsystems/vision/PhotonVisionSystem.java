package team3176.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.ArrayList;
import java.util.List;
import org.littletonrobotics.junction.Logger;
import org.photonvision.EstimatedRobotPose;
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
  public static final Transform3d Robot2camera2 =
      new Transform3d(
          new Translation3d(
              Units.inchesToMeters(-10), Units.inchesToMeters(8), Units.inchesToMeters(10)),
          new Rotation3d(
              Units.degreesToRadians(2.5),
              Units.degreesToRadians(-31),
              Units.degreesToRadians(-180 + 18)));
  public static final Transform3d Robot2camera1 =
      new Transform3d(
          new Translation3d(
              Units.inchesToMeters(-10), Units.inchesToMeters(-8), Units.inchesToMeters(10)),
          new Rotation3d(
              Units.degreesToRadians(0),
              Units.degreesToRadians(-30),
              Units.degreesToRadians(-180 - 18)));
  public static final Transform3d Robot2camera3 =
      new Transform3d(
          new Translation3d(
              Units.inchesToMeters(12), Units.inchesToMeters(10), Units.inchesToMeters(10.5)),
          new Rotation3d(
              Units.degreesToRadians(0), Units.degreesToRadians(-20), Units.degreesToRadians(0)));
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
  private ArrayList<LoggedAprilPhotonCam> aprilCameras = new ArrayList<LoggedAprilPhotonCam>();

  private LoggedNotePhotonCam notecam;
  List<Pose3d> visionTargets = new ArrayList<>();

  AprilTagFieldLayout field;
  private static PhotonVisionSystem instance;
  private SimPhotonVision simInstance;
  EstimatedRobotPose currentEstimate;

  public double noteYaw;
  public double notePitch;
  public boolean seeNote;

  private PhotonVisionSystem() {

    notecam = new LoggedNotePhotonCam();
    aprilCameras.add(new LoggedAprilPhotonCam("camera1", Robot2camera1));
    aprilCameras.add(new LoggedAprilPhotonCam("camera2", Robot2camera2));
    aprilCameras.add(new LoggedAprilPhotonCam("camera3", Robot2camera3));
    try {
      field = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
      field.setOrigin(OriginPosition.kBlueAllianceWallRightSide);
    } catch (Exception e) {
      System.out.println("woops can't load the field");
    }
    if (Constants.getMode() == Mode.SIM) {
      simInstance = new SimPhotonVision(aprilCameras, field);
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
    notecam.periodic();
    noteYaw = notecam.noteYaw;
    notePitch = notecam.notePitch;
    seeNote = notecam.seeNote;

    visionTargets.clear();
    // Drivetrain.getInstance().visionPose3d = cameras.get(0).getPoseEstimates().get(0)
    for (LoggedAprilPhotonCam c : aprilCameras) {
      c.periodic();
      visionTargets.addAll(c.getCamera2Target());
    }
    Logger.recordOutput(
        "photonvision/visionTargets", visionTargets.toArray(new Pose3d[visionTargets.size()]));
  }
}
