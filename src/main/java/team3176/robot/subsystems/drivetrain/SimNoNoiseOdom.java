package team3176.robot.subsystems.drivetrain;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import java.util.ArrayList;
import org.littletonrobotics.junction.Logger;

public class SimNoNoiseOdom {
  private SwerveDriveOdometry odomNoNoise;
  private ArrayList<SwervePod> pods;
  private Rotation2d wheelOnlyHeading;

  public SimNoNoiseOdom(ArrayList<SwervePod> pods) {
    this.pods = pods;
    System.out.println("pods size " + this.pods.size());
    wheelOnlyHeading = new Rotation2d();
    odomNoNoise =
        new SwerveDriveOdometry(
            Drivetrain.kinematics,
            new Rotation2d(),
            new SwerveModulePosition[] {
              new SwerveModulePosition(),
              new SwerveModulePosition(),
              new SwerveModulePosition(),
              new SwerveModulePosition()
            },
            new Pose2d(0.0, 0.0, new Rotation2d()));
  }

  public SwerveModulePosition[] getSwerveModulePositionsSimNoNoise() {
    SwerveModulePosition[] positions = new SwerveModulePosition[pods.size()];
    for (int i = 0; i < pods.size(); ++i) {
      positions[i] = pods.get(i).getPositionSimNoNoise();
    }
    return positions;
  }

  public void resetSimPose(Pose2d p) {
    odomNoNoise.resetPosition(p.getRotation(), getSwerveModulePositionsSimNoNoise(), p);
  }

  public Pose2d getPoseTrue() {
    return odomNoNoise.getPoseMeters();
  }

  public void update() {
    SwerveModulePosition[] deltas = new SwerveModulePosition[4];
    for (int i = 0; i < pods.size(); i++) {
      deltas[i] = pods.get(i).getDeltaSimNoNoise();
    }
    Twist2d twist = Drivetrain.kinematics.toTwist2d(deltas);
    wheelOnlyHeading = odomNoNoise.getPoseMeters().exp(twist).getRotation();
    odomNoNoise.update(wheelOnlyHeading, getSwerveModulePositionsSimNoNoise());
    Logger.recordOutput("Drive/PoseSimNoNoise", getPoseTrue());
  }
}
