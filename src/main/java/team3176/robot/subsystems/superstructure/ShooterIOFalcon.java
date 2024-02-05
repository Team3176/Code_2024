// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team3176.robot.subsystems.superstructure;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.hardware.CANcoder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import team3176.robot.constants.SuperStructureConstants;
import team3176.robot.constants.Hardwaremap;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

public class ShooterIOFalcon implements ShooterIO{
  
  private TalonFX wheelPortController = new TalonFX(Hardwaremap.shooterWheelPort_CID, Hardwaremap.shooter_CBN);
  private TalonFXConfiguration wheelPortConfigs;
  private TalonFX wheelStarbrdController;
  private TalonFXConfiguration wheelStarbrdConfigs;
  private TalonFX pivotController;
  private final VelocityVoltage m_voltageVelocity = new VelocityVoltage(0, 0, true, 0, 0, false, false, false);
  private TalonFXConfiguration pivotConfigs;
  private final NeutralOut m_brake = new NeutralOut();


  public ShooterIOFalcon() {
    wheelPortController = new TalonFX(Hardwaremap.shooterWheelPort_CID, Hardwaremap.shooter_CBN);
    wheelPortConfigs = new TalonFXConfiguration();
    wheelStarbrdController = new TalonFX(Hardwaremap.shooterWheelStarbrd_CID, Hardwaremap.shooter_CBN);
    wheelStarbrdConfigs = new TalonFXConfiguration();
    pivotController = new TalonFX(Hardwaremap.shooterPivot_CID, Hardwaremap.shooterPivot_CBN);
    pivotConfigs = new TalonFXConfiguration();


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
    applyTalonFxConfigs(pivotController, pivotConfigs);
  }

  public void setWheelPortPIDVelocity(double desiredRotationsPerSecond) {
    wheelPortController.setControl(m_voltageVelocity.withVelocity(desiredRotationsPerSecond));
  }
  
  public void brakeWheelPort() {/* Disable the motor instead */
        wheelPortController.setControl(m_brake);
  }


  @Override
  public void updateInputs(ShooterIOInputs inputs) {
      inputs.pivotPosition = Rotation2d.fromRotations(pivotController.getPosition().getValueAsDouble());
      inputs.wheelPortVelocityRadPerSec = Units.degreesToRadians(wheelPortController.getVelocity().getValueAsDouble());
      inputs.wheelStarbrdVelocityRadPerSec = Units.degreesToRadians(wheelStarbrdController.getVelocity().getValueAsDouble());
      inputs.pivotAppliedVolts = pivotController.getMotorVoltage().getValueAsDouble() * pivotController.getSupplyVoltage().getValueAsDouble();
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
    //to be implemented
  }
}

