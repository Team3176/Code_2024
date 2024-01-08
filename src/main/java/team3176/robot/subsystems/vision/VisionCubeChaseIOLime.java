package team3176.robot.subsystems.vision;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Pose3d;
import team3176.robot.subsystems.drivetrain.LimelightHelpers;

public class VisionCubeChaseIOLime implements VisionCubeChaseIO {
  public static final String rfov = "limelight-three";
  //public static final String lfov = "lfov-limelight";
    /** Updates the set of loggable inputs. */
  public void updateInputs(VisionCubeChaseInputs inputs) {
    //inputs.rfovBlue = LimelightHelpers.getBotPose3d_wpiBlue(rfov);
    //inputs.rfovRed = LimelightHelpers.getBotPose3d_wpiRed(rfov);
    inputs.rLatency = LimelightHelpers.getLatency_Capture(rfov) + LimelightHelpers.getLatency_Pipeline(rfov);
    //inputs.rNumTags = LimelightHelpers.getLatestResults(rfov).targetingResults.targets_Fiducials.length;
    inputs.rValid = LimelightHelpers.getTV(rfov);
    inputs.rTx = LimelightHelpers.getTX(rfov);
  }
}
