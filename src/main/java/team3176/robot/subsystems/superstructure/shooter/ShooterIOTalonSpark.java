// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team3176.robot.subsystems.superstructure.shooter;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkBase.SoftLimitDirection;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.SparkRelativeEncoder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import team3176.robot.constants.Hardwaremap;

public class ShooterIOTalonSpark implements ShooterIO {

  private TalonFX wheelLeftController =
      new TalonFX(Hardwaremap.shooterWheelLeft_CID, Hardwaremap.shooterWheelUpper_CBN);
  private TalonFX wheelRightController =
      new TalonFX(Hardwaremap.shooterWheelRight_CID, Hardwaremap.shooterWheelLower_CBN);

  /* private TalonFX pivotController =
       new TalonFX(Hardwaremap.shooterPivot_CID, Hardwaremap.shooterPivot_CBN);
  */
  private CANSparkFlex pivotShooter =
      new CANSparkFlex(Hardwaremap.shooterPivot_CID, MotorType.kBrushless);
  private SparkPIDController m_PidController = pivotShooter.getPIDController();
  public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput;

  // private CANcoder cancoder = new CANcoder(0);
  private RelativeEncoder pivotEncoder =
      pivotShooter.getEncoder(SparkRelativeEncoder.Type.kQuadrature, 7168);
  private CANcoderConfiguration toApply = new CANcoderConfiguration();
  // private StatusSignal CANcoderPosition = cancoder.getPosition();
  final VelocityVoltage flywheelVelocity = new VelocityVoltage(0);

  private TalonFXConfiguration configsWheelLeft = new TalonFXConfiguration();
  private TalonFXConfiguration configsWheelRight = new TalonFXConfiguration();

  private DigitalInput lowerLimitSwitch;
  private DigitalInput upperLimitSwitch;

  private final StatusSignal<Double> leftAppliedVolts;
  private final StatusSignal<Double> rightAppliedVolts;
  private final StatusSignal<Double> leftCurrentAmpsStator;
  private final StatusSignal<Double> rightCurrentAmpsStator;
  private final StatusSignal<Double> leftCurrentAmpsSupply;
  private final StatusSignal<Double> rightCurrentAmpsSupply;
  private final StatusSignal<Double> leftVelocity;
  private final StatusSignal<Double> rightVelocity;
  private double leftSetpoint;
  private double rightSetpoint;

  public ShooterIOTalonSpark() {
    pivotShooter.restoreFactoryDefaults();
    pivotEncoder.setPosition(0.0);
    pivotShooter.setSmartCurrentLimit(80);
    pivotShooter.setIdleMode(IdleMode.kBrake);
    pivotShooter.setInverted(true);
    pivotShooter.burnFlash();
    pivotShooter.enableSoftLimit(SoftLimitDirection.kReverse, true);
    pivotShooter.setSoftLimit(SoftLimitDirection.kReverse, 0);

    lowerLimitSwitch = new DigitalInput(Hardwaremap.shooterPivotLower_DIO);
    upperLimitSwitch = new DigitalInput(Hardwaremap.shooterPivotUpper_DIO);
    /*-------------------------------- Private instance variables ---------------------------------*/

    configsWheelLeft.Slot0.kP = 0.1;
    configsWheelLeft.Slot0.kI = 0.0;
    configsWheelLeft.Slot0.kD = 0.0000;
    configsWheelLeft.Slot0.kV = 0.12;
    configsWheelLeft.Voltage.PeakForwardVoltage = 12;
    configsWheelLeft.Voltage.PeakReverseVoltage = -12;
    // configsWheelUpper.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    configsWheelLeft.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    configsWheelLeft.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    configsWheelLeft.CurrentLimits.SupplyCurrentLimit = 60;
    configsWheelLeft.CurrentLimits.SupplyCurrentLimitEnable = true;
    configsWheelLeft.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 0.5;

    configsWheelRight.Slot0.kP = 0.1;
    configsWheelRight.Slot0.kI = 0.0;
    configsWheelRight.Slot0.kD = 0.0000;
    configsWheelRight.Slot0.kV = 0.12;
    configsWheelRight.Voltage.PeakForwardVoltage = 12;
    configsWheelRight.Voltage.PeakReverseVoltage = -12;
    configsWheelRight.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    configsWheelRight.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    configsWheelRight.CurrentLimits.SupplyCurrentLimit = 60;
    configsWheelRight.CurrentLimits.SupplyCurrentLimitEnable = true;
    configsWheelRight.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 0.5;

    m_PidController.setOutputRange(kMinOutput, kMaxOutput);

    wheelLeftController.getConfigurator().apply(configsWheelLeft);
    wheelRightController.getConfigurator().apply(configsWheelRight);

    leftAppliedVolts = wheelLeftController.getMotorVoltage();
    rightAppliedVolts = wheelRightController.getMotorVoltage();
    leftCurrentAmpsStator = wheelLeftController.getStatorCurrent();
    rightCurrentAmpsStator = wheelRightController.getStatorCurrent();
    leftCurrentAmpsSupply = wheelLeftController.getSupplyCurrent();
    rightCurrentAmpsSupply = wheelRightController.getSupplyCurrent();
    leftVelocity = wheelLeftController.getVelocity();
    rightVelocity = wheelRightController.getVelocity();
    // leftError = wheelLeftController.getClosedLoopReference();
    // rightError = wheelRightController.getClosedLoopReference();

    BaseStatusSignal.setUpdateFrequencyForAll(
        50,
        leftAppliedVolts,
        rightAppliedVolts,
        leftCurrentAmpsStator,
        rightCurrentAmpsStator,
        rightCurrentAmpsSupply,
        leftCurrentAmpsSupply,
        leftVelocity,
        rightVelocity);

    wheelLeftController.optimizeBusUtilization();
    wheelRightController.optimizeBusUtilization();

    // }
  }

  /*-------------------------------- Generic Subsystem Functions --------------------------------*/

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    BaseStatusSignal.refreshAll(
        leftAppliedVolts,
        rightAppliedVolts,
        leftCurrentAmpsStator,
        rightCurrentAmpsStator,
        rightCurrentAmpsSupply,
        leftCurrentAmpsSupply,
        leftVelocity,
        rightVelocity);
    inputs.pivotPosition = Rotation2d.fromRotations(pivotEncoder.getPosition() * 18 / 255);
    inputs.pivotAppliedVolts = pivotShooter.getAppliedOutput() * pivotShooter.getBusVoltage();
    inputs.wheelLeftVelocityRadPerSec = Units.rotationsToRadians(leftVelocity.getValue());
    inputs.wheelRightVelocityRadPerSec = Units.rotationsToRadians(rightVelocity.getValue());

    inputs.leftWheelReference = leftSetpoint;
    inputs.rightWheelReference = rightSetpoint;

    inputs.wheelLeftAppliedVolts = leftAppliedVolts.getValue();
    inputs.wheelRightAppliedVolts = rightAppliedVolts.getValue();

    inputs.lowerLimitSwitch = !lowerLimitSwitch.get();
    inputs.upperLimitSwitch = !upperLimitSwitch.get();

    inputs.wheelRightAmpsStator = rightCurrentAmpsStator.getValueAsDouble();
    inputs.wheelLeftAmpsStator = leftCurrentAmpsStator.getValueAsDouble();
    inputs.wheelLeftAmpsSupply = leftCurrentAmpsSupply.getValue();
    inputs.wheelRightAmpsSupply = rightCurrentAmpsSupply.getValue();

    inputs.pivotAmpsStator = pivotShooter.getOutputCurrent();
  }

  @Override
  public void setPivotVoltage(double voltage) {
    pivotShooter.setVoltage(voltage);
  }

  @Override
  public void setFlywheelVelocity(double velocity) {
    wheelRightController.setControl(flywheelVelocity.withVelocity(velocity));
    wheelLeftController.setControl(flywheelVelocity.withVelocity(velocity));
    leftSetpoint = velocity;
    rightSetpoint = velocity;
  }

  @Override
  public void setFlywheelLeftVelocity(double velocity) {
    wheelLeftController.setControl(flywheelVelocity.withVelocity(velocity));
    leftSetpoint = velocity;
  }

  @Override
  public void setFlywheelRightVelocity(double velocity) {
    wheelRightController.setControl(flywheelVelocity.withVelocity(velocity));
    rightSetpoint = velocity;
  }

  @Override
  public void setWheelLeftVoltage(double velocity) {
    wheelLeftController.set(velocity);
  }

  @Override
  public void setWheelRightVoltage(double velocity) {
    wheelRightController.set(velocity);
  }

  @Override
  public void reset() {
    // to be implemented
  }
}
