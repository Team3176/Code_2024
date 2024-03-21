// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team3176.robot.subsystems.superstructure.conveyor;

import org.littletonrobotics.junction.AutoLog;

/** Template hardware interface for a closed loop subsystem. */
public interface ConveyorIO {
  /** Contains all of the input data received from hardware. */
  @AutoLog
  public static class ConveyorIOInputs {

    public double WheelVelocity = 0.0;
    public double appliedVolts = 0.0;
    public double ampsStator = 0.0;
    public double laserDist1 = 0.0;
    public double laserDist2 = 0.0;
    // constructor if needed for some inputs
    ConveyorIOInputs() {}
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(ConveyorIOInputs inputs) {}

  public default void setController(double voltage) {}
}
