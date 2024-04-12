// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team3176.robot.subsystems.superstructure.transfer;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.util.Units;
import team3176.robot.constants.Hardwaremap;

public class TransferIOTalon implements TransferIO {

  private TalonFX transferController =
      new TalonFX(Hardwaremap.shooterTransfer_CID, Hardwaremap.shooterTransfer_CBN);

  private TalonFXConfiguration configs = new TalonFXConfiguration();
  private StatusSignal<Double> velocity;
  private StatusSignal<Double> volts;
  private StatusSignal<Double> ampsStator;
  private StatusSignal<Double> ampsSupply;

  public TransferIOTalon() {
    configs.CurrentLimits.SupplyCurrentLimit = 50;
    configs.CurrentLimits.SupplyCurrentLimitEnable = true;
    transferController.getConfigurator().apply(configs);
    velocity = transferController.getVelocity();
    volts = transferController.getMotorVoltage();
    ampsStator = transferController.getStatorCurrent();
    ampsSupply = transferController.getSupplyCurrent();
    BaseStatusSignal.setUpdateFrequencyForAll(50, velocity, volts, ampsStator, ampsSupply);
    transferController.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(TransferIOInputs inputs) {
    BaseStatusSignal.refreshAll(velocity, volts, ampsStator, ampsSupply);
    inputs.transferWheelVelocity = Units.rotationsToRadians(velocity.getValue());
    inputs.transferAppliedVolts = volts.getValue();
    inputs.ampsStator = ampsStator.getValue();
    inputs.ampsSupply = ampsSupply.getValue();
  }

  @Override
  public void setTransferController(double velocity) {
    transferController.set(velocity);
  }
}
