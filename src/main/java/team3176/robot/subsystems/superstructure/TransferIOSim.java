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
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import team3176.robot.constants.SuperStructureConstants;
import team3176.robot.Constants;

/** Template hardware interface for a closed loop subsystem. */
public class TransferIOSim implements TransferIO{
  
  private SingleJointedArmSim shooterSim;
  private double appliedVolts;
  public TransferIOSim() {
    shooterSim = new SingleJointedArmSim(DCMotor.getNEO(1), 75, 0.5, 0.7, -1.0*Math.PI, 3.14,true,0.0);
  }
  /** Updates the set of loggable inputs. */
  @Override
  public void updateInputs(TransferIOInputs inputs) {
    shooterSim.update(Constants.LOOP_PERIODIC_SECS);
    inputs.Position = Units.radiansToDegrees(shooterSim.getAngleRads()) + 90 + SuperStructureConstants.TRANSFER_SIM_OFFSET;
    inputs.VelocityRadPerSec = shooterSim.getVelocityRadPerSec();
    inputs.AppliedVolts = appliedVolts;
    inputs.CurrentAmps = new double[] {shooterSim.getCurrentDrawAmps()};
    inputs.TempCelcius = new double[] {0.0};
    Logger.recordOutput("Arm/SimPos",shooterSim.getAngleRads());
  }
  @Override
  public void set(double percentOuput) {
    if(DriverStation.isEnabled()) {
      appliedVolts = percentOuput * 12;
    } else {
      appliedVolts = 0.0;
    }
    appliedVolts = MathUtil.clamp(appliedVolts,-12,12);
    shooterSim.setInputVoltage(appliedVolts);
  }
}

