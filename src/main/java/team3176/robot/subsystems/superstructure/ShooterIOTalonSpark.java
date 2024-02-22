// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team3176.robot.subsystems.superstructure;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import team3176.robot.constants.Hardwaremap;

public class ShooterIOTalonSpark implements ShooterIO {

  private TalonFX wheelUpperController =
      new TalonFX(Hardwaremap.shooterWheelUpper_CID, Hardwaremap.shooterWheelUpper_CBN);
  private TalonFX wheelLowerController =
      new TalonFX(Hardwaremap.shooterWheelLower_CID, Hardwaremap.shooterWheelLower_CBN);
  private TalonFX wheelLowerController2 =
      new TalonFX(Hardwaremap.shooterWheelLower_CID2, Hardwaremap.shooterWheelLower_CBN2);
  /* private TalonFX pivotController =
       new TalonFX(Hardwaremap.shooterPivot_CID, Hardwaremap.shooterPivot_CBN);
  */
  private CANSparkFlex pivotShooter =
      new CANSparkFlex(Hardwaremap.shooterPivot_CID, MotorType.kBrushless);
  private SparkPIDController m_PidController = pivotShooter.getPIDController();
  private RelativeEncoder m_encoder = pivotShooter.getEncoder();
  public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput;

  private CANcoder cancoder = new CANcoder(0);
  private CANcoderConfiguration toApply = new CANcoderConfiguration();
  private StatusSignal CANcoderPosition = cancoder.getPosition();

  private TalonFXConfiguration configsWheelUpper = new TalonFXConfiguration();
  private TalonFXConfiguration configsWheelLower = new TalonFXConfiguration();
  private TalonFXConfiguration configsWheelLower2 = new TalonFXConfiguration();

  private DigitalInput lowerLimitSwitch;

  public ShooterIOTalonSpark() {
    pivotShooter.restoreFactoryDefaults();
    pivotShooter.getEncoder().setPosition(0.0);
    pivotShooter.setSmartCurrentLimit(10);
    pivotShooter.setIdleMode(IdleMode.kBrake);

    lowerLimitSwitch = new DigitalInput(1);
    /*-------------------------------- Private instance variables ---------------------------------*/

    configsWheelUpper.Slot0.kP = 0.01;
    configsWheelUpper.Slot0.kI = 0.0;
    configsWheelUpper.Slot0.kD = 0.0000;
    configsWheelUpper.Slot0.kV = 0.1;
    configsWheelUpper.Voltage.PeakForwardVoltage = 8;
    configsWheelUpper.Voltage.PeakReverseVoltage = -8;

    configsWheelLower.Slot0.kP = 0.01;
    configsWheelLower.Slot0.kI = 0.0;
    configsWheelLower.Slot0.kD = 0.0000;
    configsWheelLower.Slot0.kV = 0.11;
    configsWheelLower.Voltage.PeakForwardVoltage = 8;
    configsWheelLower.Voltage.PeakReverseVoltage = -8;

    configsWheelLower2.Slot0.kP = 0.01;
    configsWheelLower2.Slot0.kI = 0.0;
    configsWheelLower2.Slot0.kD = 0.0000;
    configsWheelLower2.Slot0.kV = 0.11;
    configsWheelLower2.Voltage.PeakForwardVoltage = 8;
    configsWheelLower2.Voltage.PeakReverseVoltage = -8;

    kP = 0.1;
    kI = 0;
    kD = 0.005;
    kIz = 0;
    kFF = 0.0000;
    kMaxOutput = 0.1;
    kMinOutput = -0.1;

    // set PID coefficients
    m_PidController.setP(kP);
    m_PidController.setI(kI);
    m_PidController.setD(kD);
    m_PidController.setIZone(kIz);
    m_PidController.setFF(kFF);
    m_PidController.setOutputRange(kMinOutput, kMaxOutput);

    // double p = SmartDashboard.getNumber("P Gain", 0);
    // double i = SmartDashboard.getNumber("I Gain", 0);
    // double d = SmartDashboard.getNumber("D Gain", 0);
    // double iz = SmartDashboard.getNumber("I Zone", 0);
    // double ff = SmartDashboard.getNumber("Feed Forward", 0);
    // double max = SmartDashboard.getNumber("Max Output", 0);
    // double min = SmartDashboard.getNumber("Min Output", 0);

    // if ((p != kP)) {
    //   m_PidController.setP(p);
    //   kP = p;
    // }
    // if ((i != kI)) {
    //   m_PidController.setI(i);
    //   kI = i;
    // }
    // if ((d != kD)) {
    //   m_PidController.setD(d);
    //   kD = d;
    // }
    // if ((iz != kIz)) {
    //   m_PidController.setIZone(iz);
    //   kIz = iz;
    // }
    // if ((ff != kFF)) {
    //   m_PidController.setFF(ff);
    //   kFF = ff;
    // }
    // if ((max != kMaxOutput) || (min != kMinOutput)) {

    m_PidController.setOutputRange(kMinOutput, kMaxOutput);

    wheelUpperController.getConfigurator().apply(configsWheelUpper);
    wheelLowerController.getConfigurator().apply(configsWheelLower);
    wheelLowerController2.getConfigurator().apply(configsWheelLower2);
    // }
  }

  /*-------------------------------- Generic Subsystem Functions --------------------------------*/

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    inputs.pivotPosition = Rotation2d.fromRotations(pivotShooter.getEncoder().getPosition());

    inputs.wheelUpperVelocityRadPerSec =
        Units.rotationsToRadians(wheelUpperController.getVelocity().getValue());
    inputs.wheelLowerVelocityRadPerSec =
        Units.rotationsToRadians(wheelLowerController.getVelocity().getValue());
    inputs.wheelLowerVelocityRadPerSec2 =
        Units.rotationsToRadians(wheelLowerController2.getVelocity().getValue());
    inputs.wheelUpperAppliedVolts = wheelUpperController.getMotorVoltage().getValue();
    inputs.wheelLowerAppliedVolts = wheelLowerController.getMotorVoltage().getValue();
    inputs.wheelLowerAppliedVolts2 = wheelLowerController2.getMotorVoltage().getValue();
    inputs.lowerLimitSwitch = !lowerLimitSwitch.get();
  }

  public void applyTalonFxConfigs(TalonFX controller, TalonFXConfiguration configs) {
    StatusCode status = StatusCode.StatusCodeNotInitialized;
    for (int i = 0; i < 5; ++i) {
      status = controller.getConfigurator().apply(configs);
      System.out.println("Applied configs to: " + controller.getDeviceID());
      if (status.isOK()) break;
    }
    if (!status.isOK()) {
      System.out.println("Could not apply configs, error code: " + status.toString());
    }
  }

  /*-------------------------------- Custom Public Functions --------------------------------*/
  @Override
  public void setPivotVoltage(double voltage) {
    pivotShooter.setVoltage(voltage);
  }

  @Override
  public void setVelocityVoltage(double velocity) {
    wheelLowerController.set(velocity);
    wheelUpperController.set(velocity);
    wheelLowerController2.set(velocity);
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
  public void setWheelLowerVoltage2(double velocity) {
    wheelLowerController2.set(velocity);
  }

  // @Override
  // public void setShooterPivotPID(int Position) {
  //   m_PidController.setReference(Position, CANSparkFlex.ControlType.kPosition);
  // }

  // @Override
  // public void setShooterPivotVoltage(int voltage) {
  //   m_PidController.setReference(7, CANSparkMax.ControlType.kVoltage);
  // }

  @Override
  public void reset() {
    // to be implemented
  }

  /*---------------------------------- Custom Private Functions ---------------------------------*/

}
