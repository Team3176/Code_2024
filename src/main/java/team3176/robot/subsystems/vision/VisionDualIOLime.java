package team3176.robot.subsystems.vision;



import team3176.robot.subsystems.drivetrain.LimelightHelpers;

public class VisionDualIOLime implements VisionDualIO {
  public static final String RFOV = "rfov-limelight";
  public static final String LFOV = "lfov-limelight";
    /** Updates the set of loggable inputs. */
  @Override
  public void updateInputs(VisionDualInputs inputs) {
    inputs.rfovBlue = LimelightHelpers.getBotPose3d_wpiBlue(RFOV);
    inputs.rfovRed = LimelightHelpers.getBotPose3d_wpiRed(RFOV);
    inputs.lfovBlue = LimelightHelpers.getBotPose3d_wpiBlue(LFOV);
    inputs.lfovRed = LimelightHelpers.getBotPose3d_wpiRed(LFOV);
    inputs.lLatency = LimelightHelpers.getLatency_Capture(LFOV) + LimelightHelpers.getLatency_Pipeline(LFOV);
    inputs.rLatency = LimelightHelpers.getLatency_Capture(RFOV) + LimelightHelpers.getLatency_Pipeline(RFOV);
    //inputs.rNumTags = LimelightHelpers.getLatestResults(RFOV).targetingResults.targets_Fiducials.length;
    //inputs.lNumTags = LimelightHelpers.getLatestResults(LFOV).targetingResults.targets_Fiducials.length;
    inputs.rValid = LimelightHelpers.getTV(RFOV);
    inputs.lValid = LimelightHelpers.getTV(LFOV);
  }
}
