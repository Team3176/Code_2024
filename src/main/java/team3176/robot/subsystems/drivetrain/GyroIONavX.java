package team3176.robot.subsystems.drivetrain;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.SPI;
import java.util.OptionalDouble;
import java.util.Queue;

public class GyroIONavX implements GyroIO {
  AHRS navX;
  private final Queue<Double> yawPositionQueue;
  private final Queue<Double> yawTimestampQueue;

  public GyroIONavX() {
    navX = new AHRS(SPI.Port.kMXP);

    yawTimestampQueue = SparkMaxOdometryThread.getInstance().makeTimestampQueue();
    yawPositionQueue =
        SparkMaxOdometryThread.getInstance()
            .registerSignal(
                () -> {
                  boolean valid = navX.isConnected();
                  if (valid) {
                    return OptionalDouble.of(navX.getRotation2d().getDegrees());
                  } else {
                    return OptionalDouble.empty();
                  }
                });
  }

  @Override
  public void updateInputs(GyroIOInputs inputs) {
    inputs.isConnected = navX.isConnected();
    inputs.pitch = navX.getPitch();
    inputs.roll = navX.getRoll();
    inputs.yaw = navX.getYaw();
    inputs.rotation2d = navX.getRotation2d();
    inputs.odometryYawTimestamps =
        yawTimestampQueue.stream().mapToDouble((Double value) -> value).toArray();
    inputs.odometryYawPositions =
        yawPositionQueue.stream()
            .map((Double value) -> Rotation2d.fromDegrees(value))
            .toArray(Rotation2d[]::new);
    yawTimestampQueue.clear();
    yawPositionQueue.clear();
  }

  @Override
  public void reset() {
    navX.reset();
  }
}
