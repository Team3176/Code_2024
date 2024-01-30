// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team3176.robot.subsystems.superstructure;

import org.littletonrobotics.junction.Logger;


import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import team3176.robot.constants.SuperStructureConstants;
import team3176.robot.Constants;

/** Template hardware interface for a closed loop subsystem. */
public class IntakeIOSim implements IntakeIO{
  
  private FlywheelSim intakeSim;
  private double appliedVolts;
  public IntakeIOSim() {
    intakeSim = new FlywheelSim(DCMotor.getNEO(1), 2.0, 0.1);
  }
  /** Updates the set of loggable inputs. */
  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    intakeSim.update(Constants.LOOP_PERIODIC_SECS);
    inputs.VelocityRadPerSec = intakeSim.getAngularVelocityRadPerSec();
    inputs.AppliedVolts = appliedVolts;
  }
  @Override
  public void set(double percentOuput) {
    if(DriverStation.isEnabled()) {
      appliedVolts = percentOuput * 12;
    } else {
      appliedVolts = 0.0;
    }
    appliedVolts = MathUtil.clamp(appliedVolts,-12,12);
    intakeSim.setInputVoltage(appliedVolts);
  }
}

