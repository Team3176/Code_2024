package team3176.robot.subsystems.drivetrain;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import org.littletonrobotics.junction.Logger;
import team3176.robot.constants.SwervePodHardwareID;

public class SwervePodIOFalconSpark implements SwervePodIO {
  private static final double AZIMUTH_GEAR_RATIO =
      70.0 / 1.0; // Is the Versa gearbox btwn motor & encoder
  public static final double THRUST_GEAR_RATIO = (14.0 / 22.0) * (15.0 / 45.0);

  public static final double AZIMUTH_ENCODER_UNITS_PER_REVOLUTION = 4096;
  public static final double THRUST_ENCODER_UNITS_PER_REVOLUTION = 2048;
  private int id;
  private CANSparkMax turnSparkMax;
  final VelocityVoltage thrustVelocity = new VelocityVoltage(0.0);
  private TalonFX thrustFalcon;
  private CANcoder azimuthEncoder;

  private Rotation2d offset;

  private final StatusSignal<Double> drivePosition;
  private final StatusSignal<Double> driveVelocity;
  private final StatusSignal<Double> driveAppliedVolts;
  private final StatusSignal<Double> driveCurrentStator;
  private final StatusSignal<Double> driveCurrentSupply;
  private final StatusSignal<Double> driveTemps;

  private final StatusSignal<Double> turnAbsolutePosition;

  // public static final double FEET2TICS = 12.0 * (1.0/ (DrivetrainConstants.WHEEL_DIAMETER_INCHES
  // * Math.PI)) * (1.0 /DrivetrainConstants.THRUST_GEAR_RATIO) *
  // DrivetrainConstants.THRUST_ENCODER_UNITS_PER_REVOLUTION;
  public SwervePodIOFalconSpark(SwervePodHardwareID id, int sparkMaxID) {
    this.id = id.SERIAL;
    turnSparkMax = new CANSparkMax(sparkMaxID, MotorType.kBrushless);
    thrustFalcon = new TalonFX(id.THRUST_CID, id.THRUST_CBN);
    azimuthEncoder = new CANcoder(id.CANCODER_CID, id.CANCODER_CBN);

    offset = Rotation2d.fromDegrees(id.OFFSET);
    // reset the motor controllers
    // thrustFalcon.configFactoryDefault();
    turnSparkMax.restoreFactoryDefaults();
    var thrustFalconConfig = new TalonFXConfiguration();

    thrustFalconConfig.CurrentLimits.StatorCurrentLimit = 40;
    thrustFalconConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    // thrustFalconConfig.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = 0.5;

    thrustFalconConfig.Slot0.kP = 0.1;
    thrustFalconConfig.Slot0.kI = 0.0;
    thrustFalconConfig.Slot0.kD = 0.0;
    thrustFalconConfig.Slot0.kV = 0.12;

    thrustFalcon.getConfigurator().apply(thrustFalconConfig);
    // turnSparkMax.setOpenLoopRampRate(0.0);
    turnSparkMax.setSmartCurrentLimit(25);
    turnSparkMax.setInverted(true);

    var azimuthEncoderConfig = new CANcoderConfiguration();
    azimuthEncoderConfig.MagnetSensor.AbsoluteSensorRange =
        AbsoluteSensorRangeValue.Signed_PlusMinusHalf;
    azimuthEncoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
    // TODO: convert offset values to be from -1 to 1 in revolution instead of encoder tics;
    // Comment out line below to test Akit way

    azimuthEncoder.getConfigurator().apply(azimuthEncoderConfig);

    drivePosition = thrustFalcon.getPosition();
    driveVelocity = thrustFalcon.getVelocity();
    driveAppliedVolts = thrustFalcon.getMotorVoltage();
    driveCurrentStator = thrustFalcon.getStatorCurrent();
    driveCurrentSupply = thrustFalcon.getSupplyCurrent();
    driveTemps = thrustFalcon.getDeviceTemp();

    turnAbsolutePosition = azimuthEncoder.getAbsolutePosition();
    // BaseStatusSignal.setUpdateFrequencyForAll(50.0, drivePosition);
    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0,
        drivePosition,
        driveVelocity,
        driveAppliedVolts,
        driveCurrentStator,
        driveCurrentSupply,
        driveTemps,
        turnAbsolutePosition);
    thrustFalcon.optimizeBusUtilization();
    // azimuthEncoder.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(SwervePodIOInputs inputs) {
    BaseStatusSignal.refreshAll(
        drivePosition,
        driveVelocity,
        driveAppliedVolts,
        driveCurrentStator,
        driveCurrentSupply,
        driveTemps,
        turnAbsolutePosition);
    inputs.drivePositionRad =
        Units.rotationsToRadians(drivePosition.getValueAsDouble()) * (THRUST_GEAR_RATIO);
    inputs.driveVelocityRadPerSec =
        Units.rotationsToRadians(driveVelocity.getValueAsDouble()) * (THRUST_GEAR_RATIO);
    inputs.driveAppliedVolts = driveAppliedVolts.getValueAsDouble();
    inputs.driveCurrentAmpsStator = driveCurrentStator.getValueAsDouble();
    inputs.driveTempCelcius = driveTemps.getValueAsDouble();

    inputs.turnAbsolutePositionDegrees =
        MathUtil.inputModulus(
            Rotation2d.fromRotations(turnAbsolutePosition.getValueAsDouble())
                .minus(offset)
                .getDegrees(),
            -180,
            180);
    Logger.recordOutput(
        "Drivetrain/IO/raw/rawNoOffset_enc" + id, turnAbsolutePosition.getValueAsDouble());
    Logger.recordOutput(
        "Drivetrain/IO/degreesNoOffset_enc" + id,
        Rotation2d.fromRotations(turnAbsolutePosition.getValueAsDouble()).getDegrees());

    // inputs.turnVelocityRPM = turnSparkMax.getEncoder().getVelocity();
    inputs.turnAppliedVolts = turnSparkMax.getAppliedOutput() * turnSparkMax.getBusVoltage();
    inputs.turnCurrentAmps = turnSparkMax.getOutputCurrent();
    inputs.turnTempCelcius = turnSparkMax.getMotorTemperature();
  }

  @Override
  public void setDrive(double velMetersPerSecond) {
    double velRotationsPerSec =
        velMetersPerSecond
            * (1.0 / (SwervePod.WHEEL_DIAMETER * Math.PI))
            * (1.0 / THRUST_GEAR_RATIO);
    thrustFalcon.setControl(this.thrustVelocity.withVelocity(velRotationsPerSec));
  }

  /** Run the turn motor at the specified voltage. */
  @Override
  public void setTurn(double percent) {
    turnSparkMax.set(percent);
  }

  /** Enable or disable brake mode on the drive motor. */
  @Override
  public void setDriveBrakeMode(boolean enable) {
    if (enable) {
      thrustFalcon.setNeutralMode(NeutralModeValue.Brake);
    } else {
      thrustFalcon.setNeutralMode(NeutralModeValue.Coast);
    }
  }

  /** Enable or disable brake mode on the turn motor. */
  @Override
  public void setTurnBrakeMode(boolean enable) {
    if (enable) {
      turnSparkMax.setIdleMode(IdleMode.kBrake);
    } else {
      turnSparkMax.setIdleMode(IdleMode.kCoast);
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
