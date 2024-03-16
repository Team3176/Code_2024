package team3176.robot.subsystems;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;
import team3176.robot.subsystems.superstructure.shooter.Shooter;
import team3176.robot.util.LoggedTunableNumber;

public class Visualization extends SubsystemBase {
  LoggedTunableNumber intakeAngle = new LoggedTunableNumber("intake_angle", 0.0);
  LoggedTunableNumber shooterAngle = new LoggedTunableNumber("shooter_angle", 0.0);
  Rotation2d shooterPivotRange = Shooter.UPPER_LIMIT.minus(Shooter.LOWER_LIMIT);

  @Override
  public void periodic() {
    Pose3d intake =
        new Pose3d(
            new Translation3d(0.2541, 0.0, 0.12),
            new Rotation3d(0, Units.degreesToRadians(intakeAngle.get()), 0));
    Pose3d shooter =
        new Pose3d(
            new Translation3d(-0.01, 0.0, 0.4309),
            new Rotation3d(
                0, Shooter.getInstance().getAngle().minus(shooterPivotRange).getRadians(), 0));
    Pose3d[] components = {intake, shooter};
    Logger.recordOutput("component_vis", components);
  }
}
