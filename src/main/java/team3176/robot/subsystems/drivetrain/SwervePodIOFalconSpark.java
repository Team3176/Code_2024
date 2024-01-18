package team3176.robot.subsystems.drivetrain;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import team3176.robot.constants.SwervePodHardwareID;

public class SwervePodIOFalconSpark implements SwervePodIO {
  private static final double AZIMUTH_GEAR_RATIO =
      70.0 / 1.0; // Is the Versa gearbox btwn motor & encoder
  public static final double THRUST_GEAR_RATIO = (14.0 / 22.0) * (15.0 / 45.0);

  public static final double AZIMUTH_ENCODER_UNITS_PER_REVOLUTION = 4096;
  public static final double THRUST_ENCODER_UNITS_PER_REVOLUTION = 2048;

  private CANSparkMax turnSparkMax;
  final VelocityVoltage thrustVelocity = new VelocityVoltage(0.0);
  private TalonFX thrustFalcon;
  private CANcoder azimuthEncoder;

  // public static final double FEET2TICS = 12.0 * (1.0/ (DrivetrainConstants.WHEEL_DIAMETER_INCHES
  // * Math.PI)) * (1.0 /DrivetrainConstants.THRUST_GEAR_RATIO) *
  // DrivetrainConstants.THRUST_ENCODER_UNITS_PER_REVOLUTION;
  public SwervePodIOFalconSpark(SwervePodHardwareID id, int sparkMaxID) {
    turnSparkMax = new CANSparkMax(sparkMaxID, MotorType.kBrushless);
    thrustFalcon = new TalonFX(id.THRUST_CID);
    // reset the motor controllers
    //thrustFalcon.configFactoryDefault();
    var thrustFalconConfig = new TalonFXConfiguration();
    turnSparkMax.restoreFactoryDefaults();

    thrustFalconConfig.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = 0.5;
    var thrustSlot0 = new Slot0Configs();
    thrustSlot0.kP = 0.07207; //from calculator only really trust 2 sig digits
    thrustSlot0.kI = 0.0;
    thrustSlot0.kD = 0.0;
    thrustSlot0.kV = 0.1081; //from calculator only really trust 2 sig digits
    thrustFalcon.getConfigurator().apply(thrustSlot0);
    thrustFalcon.getConfigurator().apply(thrustFalconConfig);
    // turnSparkMax.setOpenLoopRampRate(0.0);
    turnSparkMax.setSmartCurrentLimit(25);
    turnSparkMax.setInverted(true);

    azimuthEncoder = new CANcoder(id.CANCODER_CID);
    var azimuthEncoderConfig = new CANcoderConfiguration();
    azimuthEncoderConfig.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Signed_PlusMinusHalf;

    // TODO: convert offset values to be from -1 to 1 in revolution instead of encoder tics;
    azimuthEncoderConfig.MagnetSensor.MagnetOffset = id.OFFSET/ AZIMUTH_ENCODER_UNITS_PER_REVOLUTION;

    azimuthEncoder.getConfigurator().apply(azimuthEncoderConfig);
  }

  @Override
  public void updateInputs(SwervePodIOInputs inputs) {
    inputs.drivePositionRad =
        thrustFalcon.getRotorPosition().getValue()
            * (THRUST_GEAR_RATIO)
            * 2
            * Math.PI;
    inputs.driveVelocityRadPerSec =
        thrustFalcon.getRotorVelocity().getValue()
            * (THRUST_GEAR_RATIO)
            * 2
            * Math.PI;
    inputs.driveAppliedVolts = thrustFalcon.getMotorVoltage().getValue();
    inputs.driveCurrentAmpsStator = new double[] {thrustFalcon.getStatorCurrent().getValue()};
    inputs.driveCurrentAmpsSupply = new double[] {thrustFalcon.getSupplyCurrent().getValue()};
    inputs.driveTempCelcius = new double[] {thrustFalcon.getDeviceTemp().getValue()};

    inputs.turnAbsolutePositionDegrees = azimuthEncoder.getAbsolutePosition().getValue() * 360;

    inputs.turnVelocityRPM = turnSparkMax.getEncoder().getVelocity();
    inputs.turnAppliedVolts = turnSparkMax.getAppliedOutput() * turnSparkMax.getBusVoltage();
    inputs.turnCurrentAmps = new double[] {turnSparkMax.getOutputCurrent()};
    inputs.turnTempCelcius = new double[] {turnSparkMax.getMotorTemperature()};
  }

  @Override
  public void setDrive(double velMetersPerSecond) {
    double velTicsPer100ms =
        velMetersPerSecond
            * (1.0 / (SwervePod.WHEEL_DIAMETER * Math.PI))
            * (1.0 / THRUST_GEAR_RATIO)
    ;
    thrustFalcon.setControl(thrustVelocity.withVelocity(velTicsPer100ms));
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
}
