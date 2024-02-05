// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team3176.robot.subsystems.superstructure;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import team3176.robot.constants.Hardwaremap;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.hardware.CANcoder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import team3176.robot.constants.SuperStructureConstants;
import team3176.robot.constants.Hardwaremap;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
/** Template hardware interface for a closed loop subsystem. */
public class ShooterIOFalcon implements ShooterIO{
  
  private CANSparkMax pivotController;
  private CANcoder pivotEncoder;
  private TalonFX wheelPortController, wheelStarbrdController;
  private TalonFXConfiguration wheelPortConfigs, wheelStarbrdConfigs;
  private final VelocityVoltage m_voltageVelocity =
      new VelocityVoltage(0, 0, true, 0, 0, false, false, false);
  private TalonFXConfiguration pivotConfigs;
  private final NeutralOut m_brake = new NeutralOut();

  public ShooterIOFalcon() {
    pivotController = new CANSparkMax(Hardwaremap.shooterPivot_CID, MotorType.kBrushless);
    pivotController.setSmartCurrentLimit(SuperStructureConstants.ARM_CURRENT_LIMIT_A);
    pivotEncoder = new CANcoder(Hardwaremap.armEncoder_CID);
    pivotController.setOpenLoopRampRate(0.5);
    wheelPortController =
        new TalonFX(Hardwaremap.shooterWheelPort_CID, Hardwaremap.shooterWheelPort_CBN);
    wheelPortConfigs = new TalonFXConfiguration();
    wheelStarbrdController =
        new TalonFX(Hardwaremap.shooterWheelStarbrd_CID, Hardwaremap.shooterWheelStarbrd_CBN);
    wheelStarbrdConfigs = new TalonFXConfiguration();

    wheelPortConfigs.Slot0.kP = 0.001;
    wheelPortConfigs.Slot0.kI = 0.000;
    wheelPortConfigs.Slot0.kD = 0.000;
    wheelPortConfigs.Slot0.kV = 0.12;

    wheelStarbrdConfigs.Slot0.kP = 0.001;
    wheelStarbrdConfigs.Slot0.kI = 0.000;
    wheelStarbrdConfigs.Slot0.kD = 0.000;
    wheelStarbrdConfigs.Slot0.kV = 0.12;

    pivotConfigs.Slot0.kP = 0.001;
    pivotConfigs.Slot0.kI = 0.000;
    pivotConfigs.Slot0.kD = 0.000;
    pivotConfigs.Slot0.kV = 0.12;

    applyTalonFxConfigs(wheelPortController, wheelPortConfigs);
    applyTalonFxConfigs(wheelStarbrdController, wheelStarbrdConfigs);
    
  }

  public void setWheelPortPIDVelocity(double desiredRotationsPerSecond) {
    wheelPortController.setControl(m_voltageVelocity.withVelocity(desiredRotationsPerSecond));
  }

  public void brakeWheelPort() {
    /* Disable the motor instead */
    wheelPortController.setControl(m_brake);
  }

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    //TODO: garbage needed to compile nothing is correct
    inputs.pivotPosition = Rotation2d.fromRotations(pivotEncoder.getAbsolutePosition().getValueAsDouble());
    inputs.wheelPortVelocityRadPerSec = Units.degreesToRadians(pivotEncoder.getVelocity().getValueAsDouble());
    inputs.wheelStarbrdVelocityRadPerSec = Units.degreesToRadians(pivotEncoder.getVelocity().getValueAsDouble());
    inputs.pivotAppliedVolts = pivotController.getAppliedOutput() * pivotController.getBusVoltage();
  }

  public void applyTalonFxConfigs(TalonFX controller, TalonFXConfiguration configs) {
    /* Retry config apply up to 5 times, report if failure */
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

  @Override
  public void setPivotVoltage(double voltage) {
    pivotController.setVoltage(voltage);
  }

  @Override
  public void reset() {
    // to be implemented
  }
}
