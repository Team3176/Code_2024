package team3176.robot.subsystems.drivetrain;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public interface SwervePodIO {
  @AutoLog
  public static class SwervePodIOInputs {
    public double drivePositionRad = 0.0;
    public double driveVelocityRadPerSec = 0.0;
    public double driveAppliedVolts = 0.0;
    public double drivePositionSimNoNoise = 0.0;
    public double driveAmpsStator = 0.0;
    public double driveAmpsSupply = 0.0;
    public double driveTempCelcius = 0.0;

    public double turnAbsolutePositionDegrees = 0.0;
    public double turnAbsolutePositionDegreesSimNoNoise = 0.0;
    public double turnVelocityRPM = 0.0;
    public double turnAppliedVolts = 0.0;
    public double turnAmpsStator = 0.0;
    public double turnTempCelcius = 0.0;

    public double[] odometryTimestamps = new double[] {};
    public double[] odometryDrivePositionsRad = new double[] {};
    public Rotation2d[] odometryTurnPositions = new Rotation2d[] {};
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(SwervePodIOInputs inputs) {}

  /** Run the drive motor at the specified voltage. */
  public default void setDrive(double volts) {}

  /** Run the turn motor at the specified voltage. */
  public default void setTurn(double volts) {}

  public default void setOffset(Rotation2d offset) {}

  public default Rotation2d getOffset() {
    return new Rotation2d();
  }
  /** Enable or disable brake mode on the drive motor. */
  public default void setDriveBrakeMode(boolean enable) {}

  /** Enable or disable brake mode on the turn motor. */
  public default void setTurnBrakeMode(boolean enable) {}
}
