package team3176.robot.subsystems.drivetrain;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import java.util.ArrayList;

public class SimNoNoiseOdom {
  private SwerveDriveOdometry odomNoNoise;
  private ArrayList<SwervePod> pods;
  private Pose2d wheelOnlyPose;

  public SimNoNoiseOdom(ArrayList<SwervePod> pods) {
    this.pods = pods;
    System.out.println("pods size " + this.pods.size());
    wheelOnlyPose = new Pose2d();
  }

  public SwerveModulePosition[] getSwerveModulePositionsSimNoNoise() {
    SwerveModulePosition[] positions = new SwerveModulePosition[pods.size()];
    for (int i = 0; i < pods.size(); ++i) {
      positions[i] = pods.get(i).getPositionSimNoNoise();
    }
    return positions;
  }

  public void resetSimPose(Pose2d p) {
    wheelOnlyPose = p;
  }

  public Pose2d getPoseTrue() {
    return wheelOnlyPose;
  }

  public void update() {
    SwerveModulePosition[] deltas = new SwerveModulePosition[4];
    for (int i = 0; i < pods.size(); i++) {
      deltas[i] = pods.get(i).getDeltaSimNoNoise();
    }
    Twist2d twist = Drivetrain.kinematics.toTwist2d(deltas);
    wheelOnlyPose = wheelOnlyPose.exp(twist);
  }
}
