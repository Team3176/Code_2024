// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team3176.robot.subsystems.superstructure.shooter;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import team3176.robot.Constants;

/** Template hardware interface for a closed loop subsystem. */
public class ShooterIOSim implements ShooterIO {

  private FlywheelSim wheelSimUpper, wheelSimLower;
  private SingleJointedArmSim pivotSim;

  private double wheelUpperAppliedVolts, wheelLowerAppliedVolts;
  private double pivotAppliedVolts;

  public ShooterIOSim() {
    wheelSimUpper = new FlywheelSim(DCMotor.getFalcon500(1), 1.0, 0.025);
    wheelSimLower = new FlywheelSim(DCMotor.getFalcon500(1), 1.0, 0.025);
    pivotSim =
        new SingleJointedArmSim(
            DCMotor.getFalcon500(1),
            50.0,
            1.0,
            0.2,
            Units.degreesToRadians(0.0),
            Units.degreesToRadians(30),
            false,
            0.0);
  }
  /** Updates the set of loggable inputs. */
  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    wheelSimUpper.update(Constants.LOOP_PERIODIC_SECS);
    wheelSimLower.update(Constants.LOOP_PERIODIC_SECS);
    pivotSim.update(Constants.LOOP_PERIODIC_SECS);
    inputs.pivotAppliedVolts = pivotAppliedVolts;
    inputs.pivotPosition = Rotation2d.fromRadians(pivotSim.getAngleRads());
    inputs.wheelLeftVelocityRadPerSec = wheelSimUpper.getAngularVelocityRadPerSec();
    inputs.wheelRightVelocityRadPerSec = wheelSimLower.getAngularVelocityRadPerSec();
    inputs.wheelLeftAppliedVolts = wheelUpperAppliedVolts;
    inputs.wheelRightAppliedVolts = wheelLowerAppliedVolts;
  }

  @Override
  public void setWheelLeftVoltage(double voltage) {
    wheelUpperAppliedVolts = MathUtil.clamp(voltage, -12, 12);
    wheelSimUpper.setInputVoltage(wheelUpperAppliedVolts);
  }

  public void setWheelRightVoltage(double voltage) {
    wheelLowerAppliedVolts = MathUtil.clamp(voltage, -12, 12);
    wheelSimLower.setInputVoltage(wheelLowerAppliedVolts);
  }

  @Override
  public void setPivotVoltage(double voltage) {
    pivotAppliedVolts = MathUtil.clamp(voltage, -12, 12);
    pivotSim.setInputVoltage(pivotAppliedVolts);
  }
}
