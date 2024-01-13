package team3176.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import org.littletonrobotics.junction.Logger;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import team3176.robot.Constants;
import team3176.robot.Constants.Mode;
import team3176.robot.subsystems.drivetrain.Drivetrain;

public class PhotonVisionSystem extends SubsystemBase {
  // currently using an body frame that is at the center of the XY of the robot and projected down to the floor Z
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
          new Translation3d(Units.inchesToMeters(25/2-1), Units.inchesToMeters(25/2-1), Units.inchesToMeters(10)),
          new Rotation3d(Units.degreesToRadians(0), Units.degreesToRadians(-10), Units.degreesToRadians(-20)));
  public static final Transform3d Robot2camera2 =
      new Transform3d(
          new Translation3d(Units.inchesToMeters(25/2-1), Units.inchesToMeters(-25/2+1), Units.inchesToMeters(10)),
          new Rotation3d(Units.degreesToRadians(0), Units.degreesToRadians(-10), Units.degreesToRadians(20)));
  private ArrayList<LoggedPhotonCam> cameras = new ArrayList<LoggedPhotonCam>();
  AprilTagFieldLayout field;
  private static PhotonVisionSystem instance;
  private SimPhotonVision simInstance;
  EstimatedRobotPose currentEstimate;

  private PhotonVisionSystem() {
    cameras.add(new LoggedPhotonCam("camera1",Robot2camera1));
    cameras.add(new LoggedPhotonCam("camera2",Robot2camera2));
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
    //Nothing right now but maybe in the future
  }
}
