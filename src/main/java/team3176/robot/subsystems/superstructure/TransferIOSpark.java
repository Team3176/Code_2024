// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team3176.robot.subsystems.superstructure;

import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.util.Units;
import team3176.robot.constants.SuperStructureConstants;
import team3176.robot.constants.Hardwaremap;
/** Template hardware interface for a closed loop subsystem. */
public class TransferIOSpark implements TransferIO{
  
  private CANSparkMax armController;
  private CANCoder armEncoder;
  public TransferIOSpark() {
    armController = new CANSparkMax(Hardwaremap.transfer_CID, MotorType.kBrushless);
    armController.setSmartCurrentLimit(SuperStructureConstants.ARM_CURRENT_LIMIT_A);
    armEncoder = new CANCoder(Hardwaremap.armEncoder_CID);
    armController.setOpenLoopRampRate(0.5);
    armEncoder.configAbsoluteSensorRange(AbsoluteSensorRange.Unsigned_0_to_360);
    armEncoder.configMagnetOffset(SuperStructureConstants.ARM_ENCODER_OFFSET);
    armEncoder.configSensorDirection(true,100);
  }
  /** Updates the set of loggable inputs. */
  @Override
  public void updateInputs(TransferIOInputs inputs) {
    inputs.Position = armEncoder.getAbsolutePosition();
    inputs.VelocityRadPerSec = Units.degreesToRadians(armEncoder.getVelocity());
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

