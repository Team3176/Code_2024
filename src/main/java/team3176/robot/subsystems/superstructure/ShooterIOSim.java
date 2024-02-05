// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team3176.robot.subsystems.superstructure;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import team3176.robot.Constants;

/** Template hardware interface for a closed loop subsystem. */
public class ShooterIOSim implements ShooterIO {

  private FlywheelSim wheelPortSim, wheelStarbrdSim;
  private SingleJointedArmSim pivotSim;

  private double wheelPortAppliedVolts, wheelStarbrdAppliedVolts;
  private double pivotAppliedVolts;

  public ShooterIOSim() {
    wheelPortSim = new FlywheelSim(DCMotor.getFalcon500(1), 1.0, 0.025);
    wheelStarbrdSim = new FlywheelSim(DCMotor.getFalcon500(1), 1.0, 0.025);
    pivotSim =
        new SingleJointedArmSim(DCMotor.getFalcon500(1), 50.0, 1.0, 0.2, 0.0, 1.0, false, 0.0);
  }
  /** Updates the set of loggable inputs. */
  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    wheelPortSim.update(Constants.LOOP_PERIODIC_SECS);
    wheelStarbrdSim.update(Constants.LOOP_PERIODIC_SECS);
    pivotSim.update(Constants.LOOP_PERIODIC_SECS);
    inputs.pivotAppliedVolts = pivotAppliedVolts;
    inputs.pivotPosition = Rotation2d.fromRadians(pivotSim.getAngleRads());
    inputs.wheelPortVelocityRadPerSec = wheelPortSim.getAngularVelocityRadPerSec();
    inputs.wheelStarbrdVelocityRadPerSec = wheelPortSim.getAngularVelocityRadPerSec();
    inputs.wheelPortAppliedVolts = wheelPortAppliedVolts;
    inputs.wheelStarbrdAppliedVolts = wheelStarbrdAppliedVolts;
  }

  @Override
  public void setWheelPortVoltage(double voltage) {
    wheelPortAppliedVolts = MathUtil.clamp(voltage, -12, 12);
    wheelPortSim.setInputVoltage(wheelPortAppliedVolts);
  }

  @Override
  public void setWheelStarbrdVoltage(double voltage) {
    wheelStarbrdAppliedVolts = MathUtil.clamp(voltage, -12, 12);
    wheelStarbrdSim.setInputVoltage(wheelPortAppliedVolts);
  }

  @Override
  public void setPivotVoltage(double voltage) {
    pivotAppliedVolts = MathUtil.clamp(voltage, -12, 12);
    pivotSim.setInputVoltage(pivotAppliedVolts);
  }
}
