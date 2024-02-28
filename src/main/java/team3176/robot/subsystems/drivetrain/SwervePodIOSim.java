package team3176.robot.subsystems.drivetrain;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import java.util.Random;
import org.littletonrobotics.junction.Logger;
import team3176.robot.Constants;

public class SwervePodIOSim implements SwervePodIO {
  private FlywheelSim driveSim = new FlywheelSim(DCMotor.getFalcon500(1), 4.714, 0.025);
  private FlywheelSim turnSim = new FlywheelSim(DCMotor.getNeo550(1), 70.0, 0.0005);
  // private PIDController drivePID = new PIDController(.03, 0, 0.0,.045);
  private double turnAbsolutePositionRad = Math.random() * 2.0 * Math.PI;
  private double driveAppliedVolts = 0.0;
  private double turnAppliedVolts = 0.0;
  private double currentDriveSpeed = 0.0;
  Rotation2d offset = new Rotation2d();
  private Random simNoise = new Random();
  private static final double moduleErrorBound = 8.0;
  private double moduleOffsetError =
      Math.floor(simNoise.nextDouble() * moduleErrorBound) - moduleErrorBound / 2.0;
  private int id;

  public SwervePodIOSim(int id) {
    this.id = id;
  }

  @Override
  public void updateInputs(SwervePodIOInputs inputs) {
    driveSim.update(Constants.LOOP_PERIODIC_SECS);
    turnSim.update(Constants.LOOP_PERIODIC_SECS);
    double angleDiffRad =
        Units.radiansToDegrees(
            turnSim.getAngularVelocityRadPerSec() * Constants.LOOP_PERIODIC_SECS);
    turnAbsolutePositionRad += angleDiffRad;
    while (turnAbsolutePositionRad < -180) {
      turnAbsolutePositionRad += 360;
    }
    while (turnAbsolutePositionRad > 180) {
      turnAbsolutePositionRad -= 360;
    }

    double delta = (driveSim.getAngularVelocityRadPerSec() * Constants.LOOP_PERIODIC_SECS);
    inputs.drivePositionSimNoNoise += delta;
    inputs.drivePositionRad += delta + simNoise.nextGaussian(0.0, 2.0) * Math.pow(delta, 2) * 0.1;
    inputs.driveVelocityRadPerSec = driveSim.getAngularVelocityRadPerSec();
    inputs.driveAppliedVolts = driveAppliedVolts;
    inputs.driveCurrentAmpsStator = Math.abs(driveSim.getCurrentDrawAmps());
    inputs.driveTempCelcius = 0.0;
    currentDriveSpeed = driveSim.getAngularVelocityRadPerSec();

    inputs.turnAbsolutePositionDegreesSimNoNoise =
        Rotation2d.fromDegrees(turnAbsolutePositionRad).minus(offset).getDegrees();
    Logger.recordOutput(
        "Drivetrain/IO/raw/rawNoOffset_enc" + id,
        Rotation2d.fromDegrees(turnAbsolutePositionRad).getRotations());
    Logger.recordOutput("Drivetrain/IO/degreesNoOffset_enc" + id, turnAbsolutePositionRad);
    inputs.turnAbsolutePositionDegrees =
        Rotation2d.fromDegrees(turnAbsolutePositionRad).minus(offset).getDegrees()
            + moduleOffsetError;
    inputs.turnVelocityRPM = turnSim.getAngularVelocityRPM();
    inputs.turnAppliedVolts = turnAppliedVolts;
    inputs.turnCurrentAmps = Math.abs(turnSim.getCurrentDrawAmps());
    inputs.turnTempCelcius = 0.0;
  }

  @Override
  public void setDrive(double velMetersPerSecond) {
    double freeSpeedRadPerSec = 142.0;
    double driveSpeedError =
        velMetersPerSecond - (this.currentDriveSpeed / (2) * SwervePod.WHEEL_DIAMETER);
    double voltage =
        12
            * velMetersPerSecond
            * 1.0
            / (freeSpeedRadPerSec / (2 * Math.PI) * SwervePod.WHEEL_DIAMETER * Math.PI);
    driveAppliedVolts = MathUtil.clamp(voltage + driveSpeedError * 4, -12.0, 12.0);
    if (DriverStation.isEnabled()) {
      driveSim.setInputVoltage(driveAppliedVolts);
    } else {
      driveSim.setInputVoltage(0.0);
    }
  }

  /** Run the turn motor at the specified voltage. */
  @Override
  public void setTurn(double volts) {
    if (DriverStation.isEnabled()) {
      turnAppliedVolts = MathUtil.clamp(volts * 12, -12.0, 12.0);
      turnSim.setInputVoltage(turnAppliedVolts);
    } else {
      turnSim.setInput(0.0);
    }
  }

  @Override
  public Rotation2d getOffset() {
    return this.offset;
  }

  @Override
  public void setOffset(Rotation2d offset) {
    this.offset = offset;
  }
}
