// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team3176.robot.subsystems.superstructure;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import team3176.robot.constants.SuperStructureConstants;
import team3176.robot.constants.Hardwaremap;
/** Template hardware interface for a closed loop subsystem. */
public class ShooterIOFalcon implements ShooterIO{
  
  private CANSparkMax pivotController;
  private CANcoder pivotEncoder;
  private TalonFX wheelPortController, wheelStarbrdController;
  private TalonFXConfigurator wheelPortConfigs, wheelStarbrdConfigs;

  public ShooterIOFalcon() {
    pivotController = new CANSparkMax(Hardwaremap.shooter_CID, MotorType.kBrushless);
    pivotController.setSmartCurrentLimit(SuperStructureConstants.ARM_CURRENT_LIMIT_A);
    pivotEncoder = new CANcoder(Hardwaremap.armEncoder_CID);
    pivotController.setOpenLoopRampRate(0.5);
  }
  /** Updates the set of loggable inputs. */
  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    //TODO: garbage needed to compile nothing is correct
    inputs.pivotPosition = Rotation2d.fromRotations(pivotEncoder.getAbsolutePosition().getValueAsDouble());
    inputs.wheelPortVelocityRadPerSec = Units.degreesToRadians(pivotEncoder.getVelocity().getValueAsDouble());
    inputs.wheelStarbrdVelocityRadPerSec = Units.degreesToRadians(pivotEncoder.getVelocity().getValueAsDouble());
    inputs.pivotAppliedVolts = pivotController.getAppliedOutput() * pivotController.getBusVoltage();
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

