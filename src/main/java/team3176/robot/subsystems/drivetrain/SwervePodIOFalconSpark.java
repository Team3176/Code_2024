package team3176.robot.subsystems.drivetrain;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import java.util.Queue;
import org.littletonrobotics.junction.Logger;
import team3176.robot.Constants;
import team3176.robot.Constants.RobotType;
import team3176.robot.constants.SwervePodHardwareID;

public class SwervePodIOFalconSpark implements SwervePodIO {
  private static final double AZIMUTH_GEAR_RATIO =
      70.0 / 1.0; // Is the Versa gearbox btwn motor & encoder
  public static final double THRUST_GEAR_RATIO = (14.0 / 22.0) * (15.0 / 45.0);

  public static final double AZIMUTH_ENCODER_UNITS_PER_REVOLUTION = 4096;
  public static final double THRUST_ENCODER_UNITS_PER_REVOLUTION = 2048;
  private int id;
  private TalonFX turnTalonFX;
  final VelocityVoltage thrustVelocity = new VelocityVoltage(0.0);
  // private final VelocityTorqueCurrentFOC velocityTorqueCurrentFOC = new
  // VelocityTorqueCurrentFOC(0).withUpdateFreqHz(0);
  private final VelocityTorqueCurrentFOC velocityTorqueCurrentFOC = new VelocityTorqueCurrentFOC(0);
  private TalonFX thrustFalcon;
  private CANcoder azimuthEncoder;

  private Rotation2d offset;

  private final StatusSignal<Angle> drivePosition;
  private final StatusSignal<AngularVelocity> driveVelocity;
  private final StatusSignal<Voltage> driveAppliedVolts;
  private final StatusSignal<Current> driveCurrentStator;
  private final StatusSignal<Current> driveCurrentSupply;
  private final StatusSignal<Temperature> driveTemps;

  private final StatusSignal<Angle> turnPosition;
  private final StatusSignal<AngularVelocity> turnVelocity;
  private final StatusSignal<Voltage> turnAppliedVolts;
  private final StatusSignal<Current> turnCurrentStator;
  private final StatusSignal<Current> turnCurrentSupply;
  private final StatusSignal<Temperature> turnTemps;

  private final StatusSignal<Angle> turnAbsolutePosition;

  private final Queue<Double> turnPositionQueue;
  private final Queue<Double> drivePositionQueue;
  private final Queue<Double> timestampQueue;

  // public static final double FEET2TICS = 12.0 * (1.0/ (DrivetrainConstants.WHEEL_DIAMETER_INCHES
  // * Math.PI)) * (1.0 /DrivetrainConstants.THRUST_GEAR_RATIO) *
  // DrivetrainConstants.THRUST_ENCODER_UNITS_PER_REVOLUTION;
  public SwervePodIOFalconSpark(SwervePodHardwareID id, int sparkMaxID) {
    this.id = id.SERIAL;
    turnTalonFX = new TalonFX(id.AZIMUTH_CID, id.AZIMUTH_CBN);
    // turnSparkMax = new CANSparkMax(sparkMaxID, MotorType.kBrushless);
    thrustFalcon = new TalonFX(id.THRUST_CID, id.THRUST_CBN);
    azimuthEncoder = new CANcoder(id.CANCODER_CID, id.CANCODER_CBN);

    offset = Rotation2d.fromDegrees(id.OFFSET);
    // reset the motor controllers
    // thrustFalcon.configFactoryDefault();
    // turnSparkMax.restoreFactoryDefaults();
    var thrustFalconConfig = new TalonFXConfiguration();

    thrustFalconConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    thrustFalconConfig.TorqueCurrent.PeakForwardTorqueCurrent = 80;
    thrustFalconConfig.TorqueCurrent.PeakReverseTorqueCurrent = -80;
    thrustFalconConfig.ClosedLoopRamps.TorqueClosedLoopRampPeriod = 0.02;
    // thrustFalconConfig.Feedback.SensorToMechanismRatio = (1.0 / THRUST_GEAR_RATIO);
    // thrustFalconConfig.CurrentLimits.StatorCurrentLimit = 40;
    // thrustFalconConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    // thrustFalconConfig.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = 0.5;

    thrustFalconConfig.Slot0.kP = 0.2;
    thrustFalconConfig.Slot0.kI = 0.0;
    thrustFalconConfig.Slot0.kD = 0.0;
    thrustFalconConfig.Slot0.kV = 0.12;
    thrustVelocity.Slot = 0;

    thrustFalconConfig.Slot1.kP = 5.0;
    thrustFalconConfig.Slot1.kI = 0;
    thrustFalconConfig.Slot1.kD = 0.001;

    thrustFalcon.getConfigurator().apply(thrustFalconConfig);

    var turnTalonFXConfig = new TalonFXConfiguration();
    turnTalonFXConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    turnTalonFXConfig.TorqueCurrent.PeakForwardTorqueCurrent = 80;
    turnTalonFXConfig.TorqueCurrent.PeakReverseTorqueCurrent = -80;
    turnTalonFXConfig.ClosedLoopRamps.TorqueClosedLoopRampPeriod = 0.02;
    turnTalonFXConfig.Slot0.kP = 0.2;
    turnTalonFXConfig.Slot0.kI = 0.0;
    turnTalonFXConfig.Slot0.kD = 0.0;
    turnTalonFXConfig.Slot0.kV = 0.12;

    turnTalonFXConfig.Slot1.kP = 5.0;
    turnTalonFXConfig.Slot1.kI = 0;
    turnTalonFXConfig.Slot1.kD = 0.001;

    turnTalonFX.getConfigurator().apply(turnTalonFXConfig);

    // turnSparkMax.setOpenLoopRampRate(0.0);
    // turnSparkMax.setSmartCurrentLimit(25);
    // turnSparkMax.setInverted(true);

    var azimuthEncoderConfig = new CANcoderConfiguration();
    azimuthEncoderConfig.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 0.5;
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

    turnPosition = turnTalonFX.getPosition();
    turnVelocity = turnTalonFX.getVelocity();
    turnAppliedVolts = turnTalonFX.getMotorVoltage();
    turnCurrentStator = turnTalonFX.getStatorCurrent();
    turnCurrentSupply = turnTalonFX.getSupplyCurrent();
    turnTemps = turnTalonFX.getDeviceTemp();

    turnAbsolutePosition = azimuthEncoder.getAbsolutePosition();
    // BaseStatusSignal.setUpdateFrequencyForAll(50.0, drivePosition);
    BaseStatusSignal.setUpdateFrequencyForAll(
        SwervePod.ODOMETRY_FREQUENCY, drivePosition, turnAbsolutePosition);
    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0, driveVelocity, driveAppliedVolts, driveCurrentStator, driveCurrentSupply, driveTemps);
    thrustFalcon.optimizeBusUtilization();

    timestampQueue = PhoenixOdometryThread.getInstance().makeTimestampQueue();
    drivePositionQueue =
        PhoenixOdometryThread.getInstance()
            .registerSignal(thrustFalcon, thrustFalcon.getPosition());
    turnPositionQueue =
        PhoenixOdometryThread.getInstance()
            .registerSignal(azimuthEncoder, azimuthEncoder.getAbsolutePosition());
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
        turnPosition,
        turnVelocity,
        turnAppliedVolts,
        turnCurrentStator,
        turnCurrentSupply,
        turnTemps,
        turnAbsolutePosition);
    inputs.drivePositionRad =
        Units.rotationsToRadians(drivePosition.getValueAsDouble()) * (THRUST_GEAR_RATIO);
    inputs.driveVelocityRadPerSec =
        Units.rotationsToRadians(driveVelocity.getValueAsDouble()) * (THRUST_GEAR_RATIO);
    inputs.driveAppliedVolts = driveAppliedVolts.getValueAsDouble();
    inputs.driveAmpsStator = driveCurrentStator.getValueAsDouble();
    inputs.driveAmpsSupply = driveCurrentSupply.getValueAsDouble();
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

    inputs.turnVelocityRPM =
        turnVelocity.getValueAsDouble(); // TODO:  Double check the math.  Does need modifier?
    // inputs.turnVelocityRPM = turnSparkMax.getEncoder().getVelocity();
    inputs.turnAppliedVolts = turnAppliedVolts.getValueAsDouble();
    // inputs.turnAppliedVolts = turnSparkMax.getAppliedOutput() * turnSparkMax.getBusVoltage();
    inputs.turnAmpsStator = turnCurrentStator.getValueAsDouble();
    inputs.turnAmpsSupply = turnCurrentSupply.getValueAsDouble();
    inputs.turnTempCelcius = turnTemps.getValueAsDouble();

    inputs.odometryTimestamps =
        timestampQueue.stream().mapToDouble((Double value) -> value).toArray();
    inputs.odometryDrivePositionsRad =
        drivePositionQueue.stream()
            .mapToDouble((Double value) -> Units.rotationsToRadians(value) * THRUST_GEAR_RATIO)
            .toArray();
    inputs.odometryTurnPositions =
        turnPositionQueue.stream()
            .map((Double value) -> Rotation2d.fromRotations(value))
            .toArray(Rotation2d[]::new);
    timestampQueue.clear();
    drivePositionQueue.clear();
    turnPositionQueue.clear();
  }

  @Override
  public void setDrive(double velMetersPerSecond) {
    double velRotationsPerSec =
        velMetersPerSecond * (1.0 / (SwervePod.WHEEL_DIAMETER * Math.PI)) * 1.0 / THRUST_GEAR_RATIO;
    if (Constants.getRobot() == RobotType.ROBOT_2024C) {
      thrustFalcon.setControl(velocityTorqueCurrentFOC.withVelocity(velRotationsPerSec));
    } else {
      thrustFalcon.setControl(thrustVelocity.withVelocity(velRotationsPerSec));
    }
  }

  /** Run the turn motor at the specified voltage. */
  @Override
  public void setTurn(double percent) {
    turnTalonFX.set(percent);
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
      turnTalonFX.setNeutralMode(NeutralModeValue.Brake);
    } else {
      turnTalonFX.setNeutralMode(NeutralModeValue.Coast);
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
