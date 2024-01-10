package team3176.robot.subsystems.drivetrain;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;

public class GyroIONavX implements GyroIO {
  AHRS navX;

  public GyroIONavX() {
    navX = new AHRS(SPI.Port.kMXP);
  }

  @Override
  public void updateInputs(GyroIOInputs inputs) {
    inputs.pitch = navX.getPitch();
    inputs.roll = navX.getRoll();
    inputs.yaw = navX.getYaw();
    inputs.rotation2d = navX.getRotation2d();
  }

  @Override
  public void reset() {
    navX.reset();
  }
}
