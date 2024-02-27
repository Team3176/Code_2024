// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team3176.robot.subsystems.superstructure.shooter;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.CANSparkBase.IdleMode;
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

  private TalonFX wheelUpperController =
      new TalonFX(Hardwaremap.shooterWheelUpper_CID, Hardwaremap.shooterWheelUpper_CBN);
  private TalonFX wheelLowerController =
      new TalonFX(Hardwaremap.shooterWheelLower_CID, Hardwaremap.shooterWheelLower_CBN);

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

  private TalonFXConfiguration configsWheelUpper = new TalonFXConfiguration();
  private TalonFXConfiguration configsWheelLower = new TalonFXConfiguration();

  private DigitalInput lowerLimitSwitch;
  private DigitalInput upperLimitSwitch;

  public ShooterIOTalonSpark() {
    pivotShooter.restoreFactoryDefaults();
    pivotEncoder.setPosition(0.0);
    // pivotShooter.setSmartCurrentLimit(40);
    pivotShooter.setIdleMode(IdleMode.kBrake);
    pivotShooter.setInverted(true);
    pivotShooter.burnFlash();

    lowerLimitSwitch = new DigitalInput(Hardwaremap.shooterPivotLower_DIO);
    upperLimitSwitch = new DigitalInput(Hardwaremap.shooterPivotUpper_DIO);
    /*-------------------------------- Private instance variables ---------------------------------*/

    configsWheelUpper.Slot0.kP = 0.1;
    configsWheelUpper.Slot0.kI = 0.0;
    configsWheelUpper.Slot0.kD = 0.0000;
    configsWheelUpper.Slot0.kV = 0.1;
    configsWheelUpper.Voltage.PeakForwardVoltage = 12;
    configsWheelUpper.Voltage.PeakReverseVoltage = -12;
    configsWheelUpper.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    configsWheelUpper.MotorOutput.NeutralMode = NeutralModeValue.Coast;

    configsWheelLower.Slot0.kP = 0.1;
    configsWheelLower.Slot0.kI = 0.0;
    configsWheelLower.Slot0.kD = 0.0000;
    configsWheelLower.Slot0.kV = 0.11;
    configsWheelLower.Voltage.PeakForwardVoltage = 12;
    configsWheelLower.Voltage.PeakReverseVoltage = -12;
    configsWheelLower.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    configsWheelLower.MotorOutput.NeutralMode = NeutralModeValue.Coast;

    m_PidController.setOutputRange(kMinOutput, kMaxOutput);

    wheelUpperController.getConfigurator().apply(configsWheelUpper);
    wheelLowerController.getConfigurator().apply(configsWheelLower);

    // }
  }

  /*-------------------------------- Generic Subsystem Functions --------------------------------*/

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    inputs.pivotPosition = Rotation2d.fromRotations(pivotEncoder.getPosition() * 18 / 255);
    inputs.pivotAppliedVolts = pivotShooter.getAppliedOutput() * pivotShooter.getBusVoltage();
    inputs.wheelUpperVelocityRadPerSec =
        Units.rotationsToRadians(wheelUpperController.getVelocity().getValue());
    inputs.wheelLowerVelocityRadPerSec =
        Units.rotationsToRadians(wheelLowerController.getVelocity().getValue());

    inputs.wheelUpperAppliedVolts = wheelUpperController.getMotorVoltage().getValue();
    inputs.wheelLowerAppliedVolts = wheelLowerController.getMotorVoltage().getValue();

    inputs.lowerLimitSwitch = !lowerLimitSwitch.get();
    inputs.upperLimitSwitch = !upperLimitSwitch.get();
  }

  @Override
  public void setPivotVoltage(double voltage) {
    pivotShooter.setVoltage(voltage);
  }

  @Override
  public void setFlywheelVelocity(double velocity) {
    wheelLowerController.setControl(flywheelVelocity.withVelocity(velocity));
    wheelUpperController.setControl(flywheelVelocity.withVelocity(velocity));
  }

  @Override
  public void setFlywheelUpperVelocity(double velocity) {
    wheelUpperController.setControl(flywheelVelocity.withVelocity(velocity));
  }

  @Override
  public void setFlywheelLowerVelocity(double velocity) {
    wheelLowerController.setControl(flywheelVelocity.withVelocity(velocity));
  }

  @Override
  public void setWheelUpperVoltage(double velocity) {
    wheelUpperController.set(velocity);
  }

  @Override
  public void setWheelLowerVoltage(double velocity) {
    wheelLowerController.set(velocity);
  }

  @Override
  public void reset() {
    // to be implemented
  }
}
