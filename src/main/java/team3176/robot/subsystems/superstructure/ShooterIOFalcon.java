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

import edu.wpi.first.math.util.Units;
import team3176.robot.constants.SuperStructureConstants;
import team3176.robot.constants.Hardwaremap;
/** Template hardware interface for a closed loop subsystem. */
public class ShooterIOFalcon implements ShooterIO{
  
  private CANSparkMax armController;
  private CANcoder armEncoder;
  public ShooterIOFalcon() {
    armController = new CANSparkMax(Hardwaremap.shooter_CID, MotorType.kBrushless);
    armController.setSmartCurrentLimit(SuperStructureConstants.ARM_CURRENT_LIMIT_A);
    armEncoder = new CANcoder(Hardwaremap.armEncoder_CID);
    armController.setOpenLoopRampRate(0.5);
  }
  /** Updates the set of loggable inputs. */
  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    inputs.Position = armEncoder.getAbsolutePosition().getValueAsDouble();
    inputs.VelocityRadPerSec = Units.degreesToRadians(armEncoder.getVelocity().getValueAsDouble());
    inputs.AppliedVolts = armController.getAppliedOutput() * armController.getBusVoltage();
    inputs.CurrentAmps = new double[] {armController.getOutputCurrent()};
    inputs.TempCelcius = new double[] {armController.getMotorTemperature()};
  }
  @Override
  public void set(double percentOuput) {
    armController.set(percentOuput);
  }
  @Override
  public void setCoastMode(boolean isCoastMode) {
    if(isCoastMode) {
      armController.setIdleMode(IdleMode.kCoast);
    } else {
      armController.setIdleMode(IdleMode.kBrake);
    }
  }
  @Override
  public void reset() {
    //to be implemented
  }
}

