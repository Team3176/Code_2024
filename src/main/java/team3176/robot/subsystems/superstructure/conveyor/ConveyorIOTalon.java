// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team3176.robot.subsystems.superstructure.conveyor;

import au.grapplerobotics.ConfigurationFailedException;
import au.grapplerobotics.LaserCan;
import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.util.Units;
import team3176.robot.constants.Hardwaremap;

public class ConveyorIOTalon implements ConveyorIO {

  private TalonFX controller = new TalonFX(Hardwaremap.conveyor, Hardwaremap.conveyor_CBN);
  private LaserCan laserCan;
  private TalonFXConfiguration configs = new TalonFXConfiguration();
  private final StatusSignal<Double> wheelVelocity;
  private final StatusSignal<Double> appliedVolts;
  private final StatusSignal<Double> current;

  public ConveyorIOTalon() {
    laserCan = new LaserCan(Hardwaremap.LaserCan_CID);
    try {
      laserCan.setRangingMode(LaserCan.RangingMode.SHORT);
      laserCan.setRegionOfInterest(new LaserCan.RegionOfInterest(8, 8, 16, 16));
      laserCan.setTimingBudget(LaserCan.TimingBudget.TIMING_BUDGET_33MS);
    } catch (ConfigurationFailedException e) {
      System.out.println("LaserCan configuration failed");
    }
    controller.getConfigurator().apply(configs);
    wheelVelocity = controller.getVelocity();
    appliedVolts = controller.getMotorVoltage();
    current = controller.getStatorCurrent();
    BaseStatusSignal.setUpdateFrequencyForAll(50, wheelVelocity, appliedVolts, current);
    controller.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(ConveyorIOInputs inputs) {
    BaseStatusSignal.refreshAll(wheelVelocity, appliedVolts, current);
    inputs.WheelVelocity = Units.rotationsToRadians(wheelVelocity.getValue());
    inputs.appliedVolts = appliedVolts.getValue();
    inputs.ampsStator = current.getValue();
    var measurement = laserCan.getMeasurement();
    if (measurement != null) {
      inputs.laserDist1 = measurement.distance_mm;
    }
  }

  @Override
  public void setController(double velocity) {
    controller.set(velocity);
  }
}
