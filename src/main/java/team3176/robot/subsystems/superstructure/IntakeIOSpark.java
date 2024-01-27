// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team3176.robot.subsystems.superstructure;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.util.Units;
import team3176.robot.constants.SuperStructureConstants;
import team3176.robot.constants.Hardwaremap;
/** Template hardware interface for a closed loop subsystem. */
public class IntakeIOSpark implements IntakeIO{
  
  private CANSparkMax intakeController;
  private CANcoder intakeEncoder;
  public IntakeIOSpark() {
    intakeController = new CANSparkMax(Hardwaremap.intake_CID, MotorType.kBrushless);
    intakeController.setSmartCurrentLimit(SuperStructureConstants.ARM_CURRENT_LIMIT_A);
    intakeEncoder = new CANcoder(Hardwaremap.armEncoder_CID);
    intakeController.setOpenLoopRampRate(0.5);
  }
  /** Updates the set of loggable inputs. */
  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    inputs.Position = intakeEncoder.getAbsolutePosition().getValueAsDouble();
    inputs.VelocityRadPerSec = Units.degreesToRadians(intakeEncoder.getVelocity().getValueAsDouble());
    inputs.AppliedVolts = intakeController.getAppliedOutput() * intakeController.getBusVoltage();
    inputs.CurrentAmps = new double[] {intakeController.getOutputCurrent()};
    inputs.TempCelcius = new double[] {intakeController.getMotorTemperature()};
  }
  @Override
  public void set(double percentOuput) {
    intakeController.set(percentOuput);
  }
  @Override
  public void setCoastMode(boolean isCoastMode) {
    if(isCoastMode) {
      intakeController.setIdleMode(IdleMode.kCoast);
    } else {
      intakeController.setIdleMode(IdleMode.kBrake);
    }
  }
  @Override
  public void reset() {
    //to be implemented
  }
}

