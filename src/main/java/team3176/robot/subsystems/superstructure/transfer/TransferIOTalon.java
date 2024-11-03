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
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import team3176.robot.constants.Hardwaremap;

public class TransferIOTalon implements TransferIO {

  private TalonFX transferController =
      new TalonFX(Hardwaremap.shooterTransfer_CID, Hardwaremap.shooterTransfer_CBN);

  private TalonFXConfiguration configs = new TalonFXConfiguration();
  private StatusSignal<AngularVelocity> velocity;
  private StatusSignal<Voltage> volts;
  private StatusSignal<Current> ampsStator;
  private StatusSignal<Current> ampsSupply;

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
    inputs.transferWheelVelocity = Units.rotationsToRadians(velocity.getValueAsDouble());
    inputs.transferAppliedVolts = volts.getValueAsDouble();
    inputs.ampsStator = ampsStator.getValueAsDouble();
    inputs.ampsSupply = ampsSupply.getValueAsDouble();
  }

  @Override
  public void setTransferController(double velocity) {
    transferController.set(velocity);
  }
}
