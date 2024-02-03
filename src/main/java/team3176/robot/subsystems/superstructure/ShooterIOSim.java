// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team3176.robot.subsystems.superstructure;

import org.littletonrobotics.junction.Logger;


import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import team3176.robot.constants.SuperStructureConstants;
import team3176.robot.Constants;

/** Template hardware interface for a closed loop subsystem. */
public class ShooterIOSim implements ShooterIO{
  
  private FlywheelSim wheelSim;
  private SingleJointedArmSim pivotSim;

  private double wheelAppliedVolts;
  private double pivotAppliedVolts;
  public ShooterIOSim() {
    wheelSim = new FlywheelSim(DCMotor.getFalcon500(1), 1.0, 0.025);
    pivotSim = new SingleJointedArmSim(DCMotor.getFalcon500(1),50.0,1.0,0.2,0.0,1.0, false,0.0);

  }
  /** Updates the set of loggable inputs. */
  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    wheelSim.update(Constants.LOOP_PERIODIC_SECS);
    pivotSim.update(Constants.LOOP_PERIODIC_SECS);
    inputs.pivotAppliedVolts = pivotAppliedVolts;
    inputs.pivotPosition = Rotation2d.fromRadians(pivotSim.getAngleRads());
    inputs.wheelPortVelocityRadPerSec = wheelSim.getAngularVelocityRadPerSec();
    inputs.wheelStarbrdVelocityRadPerSec = wheelSim.getAngularVelocityRadPerSec();
    inputs.wheelPortAppliedVolts = wheelAppliedVolts;
    inputs.wheelStarbrdAppliedVolts = wheelAppliedVolts;
  }
  @Override
  public void setWheelPortVoltage(double voltage) {
    wheelAppliedVolts = MathUtil.clamp(voltage,-12,12);
    wheelSim.setInputVoltage(wheelAppliedVolts);
  }
  public void setWheelStarbrdVoltage(double voltage) {
    wheelAppliedVolts = MathUtil.clamp(voltage,-12,12);
    wheelSim.setInputVoltage(wheelAppliedVolts);
  }
  @Override
  public void setPivotVoltage(double voltage) {
      pivotAppliedVolts = MathUtil.clamp(voltage,-12,12);
      pivotSim.setInputVoltage(pivotAppliedVolts);
  }
}

