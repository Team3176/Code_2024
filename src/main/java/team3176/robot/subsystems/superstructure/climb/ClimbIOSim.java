// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team3176.robot.subsystems.superstructure.climb;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import org.littletonrobotics.junction.Logger;
import team3176.robot.Constants;

/** Template hardware interface for the Elevator subsystem. */
public class ClimbIOSim implements ClimbIO {

  private ElevatorSim elevatorSim;
  private double appliedVolts;

  public ClimbIOSim() {
    elevatorSim =
        new ElevatorSim(DCMotor.getFalcon500(2), 10, 5.0, 0.0381 / 2.0, 0.0, 0.75, true, 0.0);
  }
  /** Updates the set of loggable inputs. */
  @Override
  public void updateInputs(ClimbIOInputs inputs) {
    elevatorSim.update(Constants.LOOP_PERIODIC_SECS);
    inputs.leftPosition = elevatorSim.getPositionMeters();
    // inputs.VelocityRadPerSec = elevatorSim.getVelocityMetersPerSecond();
    // inputs.AppliedVolts = appliedVolts;
    // inputs.CurrentAmps = new double[] {elevatorSim.getCurrentDrawAmps()};
    // inputs.TempCelcius = new double[] {0.0};
    Logger.recordOutput("Elevator/SimPos", elevatorSim.getPositionMeters());
  }

  @Override
  public void setLeft(double voltage) {
    appliedVolts = voltage;
    appliedVolts = MathUtil.clamp(appliedVolts, -12, 12);
    elevatorSim.setInputVoltage(appliedVolts);
  }
}
