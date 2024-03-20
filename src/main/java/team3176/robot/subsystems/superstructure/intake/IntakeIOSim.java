// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team3176.robot.subsystems.superstructure.intake;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import org.littletonrobotics.junction.Logger;
import team3176.robot.Constants;
import team3176.robot.constants.SuperStructureConstants;

/** Template hardware interface for a closed loop subsystem. */
public class IntakeIOSim implements IntakeIO {

  private SingleJointedArmSim pivotSim;
  private FlywheelSim rollerSim;
  private double appliedVolts;

  public IntakeIOSim() {
    pivotSim =
        new SingleJointedArmSim(
            DCMotor.getFalcon500(1), 20, 0.5, 0.7, -1.0 * Math.PI, 3.14, true, 0.0);
    rollerSim = new FlywheelSim(DCMotor.getFalcon500(1), 1.0, 0.025);
  }
  /** Updates the set of loggable inputs. */
  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    pivotSim.update(Constants.LOOP_PERIODIC_SECS);
    rollerSim.update(Constants.LOOP_PERIODIC_SECS);
    inputs.pivotPosition =
        Units.radiansToDegrees(pivotSim.getAngleRads())
            + 90
            + SuperStructureConstants.INTAKE_PIVOT_SIM_OFFSET;
    inputs.pivotVelocityRadPerSec = pivotSim.getVelocityRadPerSec();
    inputs.pivotAppliedVolts = appliedVolts;
    inputs.pivotAmpsStator = pivotSim.getCurrentDrawAmps();
    inputs.pivotTempCelcius = 0.0;
    inputs.rollerVelocityRadPerSec = rollerSim.getAngularVelocityRadPerSec();
    inputs.rollerAppliedVolts = appliedVolts;
    inputs.rollerAmpsStator = rollerSim.getCurrentDrawAmps();
    inputs.rollerTempCelcius = 0.0;
    Logger.recordOutput("Intake/SimPivotPos", pivotSim.getAngleRads());
  }

  @Override
  public void setPivotVolts(double volts) {
    if (DriverStation.isEnabled()) {
      appliedVolts = volts;
    } else {
      appliedVolts = 0.0;
    }
    appliedVolts = MathUtil.clamp(appliedVolts, -12, 12);
    pivotSim.setInputVoltage(appliedVolts);
  }
}
