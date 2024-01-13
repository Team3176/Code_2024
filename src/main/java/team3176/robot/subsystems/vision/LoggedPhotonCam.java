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

public class LoggedPhotonCam extends SubsystemBase {
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
  private Transform3d robot2Camera =
      new Transform3d(
          new Translation3d(Units.inchesToMeters(25/2-1), Units.inchesToMeters(25/2-1), Units.inchesToMeters(10)),
          new Rotation3d(Units.degreesToRadians(0), Units.degreesToRadians(-10), Units.degreesToRadians(-20)));
  private PhotonCamera realCam;
  String name = "";
  AprilTagFieldLayout field;
  PhotonPoseEstimator estimator;
  EstimatedRobotPose currentEstimate;

  public LoggedPhotonCam(String name,Transform3d robot2Camera) {
    this.name = name;
    realCam = new PhotonCamera(name);
    this.robot2Camera = robot2Camera;
    try {
      field = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
      field.setOrigin(OriginPosition.kBlueAllianceWallRightSide);
    } catch (Exception e) {
      System.out.println("woops can't load the field");
    }
    estimator =
        new PhotonPoseEstimator(
            field,
            PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
            realCam,
            robot2Camera);
    
  }
  public Transform3d getRobot2Camera() {
    return robot2Camera;
  }
  public PhotonCamera getCamera() {
    return realCam;
  }


  @Override
  public void periodic() {
    Logger.recordOutput("photonvision/"+name+"/cameraPose", new Pose3d(Drivetrain.getInstance().getPose()).transformBy(robot2Camera));
    Pose2d currentPose = Drivetrain.getInstance().getPose();
    Pose3d current3d = new Pose3d(currentPose);
    var results = realCam.getLatestResult();
    Logger.recordOutput("photonvision/"+name+"/raw",PhotonPipelineResult.proto,results);
    if (results.hasTargets()) {
      
      Optional<EstimatedRobotPose> poseEst = estimator.update();

      ArrayList<Pose3d> targets = new ArrayList<Pose3d>();
      ArrayList<Pose3d> estimates = new ArrayList<Pose3d>();
      for (PhotonTrackedTarget t : realCam.getLatestResult().getTargets()) {
        targets.add(current3d.transformBy(robot2Camera).transformBy(t.getBestCameraToTarget()));
        estimates.add(
            PhotonUtils.estimateFieldToRobotAprilTag(
                t.getBestCameraToTarget(),
                field.getTagPose(t.getFiducialId()).get(),
                robot2Camera.inverse()));
      }

      if (poseEst.isPresent()) {
        Drivetrain.getInstance().addVisionPose(poseEst.get());
        Logger.recordOutput("photonvision/"+name+"/multitag", poseEst.get().estimatedPose);
      }
      Logger.recordOutput("photonvision/"+name+"/targetposes", targets.toArray(new Pose3d[targets.size()]));
      Logger.recordOutput(
          "photonvision/"+name+"/poseEstimates", estimates.toArray(new Pose3d[estimates.size()]));
    } else {
      Logger.recordOutput("photonvision/"+name+"/targetposes", new Pose3d[] {});
      Logger.recordOutput("photonvision/"+name+"/poseEstimates", new Pose3d[] {});
    }
    
  }
}
