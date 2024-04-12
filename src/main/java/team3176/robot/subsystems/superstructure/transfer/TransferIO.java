// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team3176.robot.subsystems.superstructure.transfer;

import org.littletonrobotics.junction.AutoLog;

/** Template hardware interface for a closed loop subsystem. */
public interface TransferIO {
  /** Contains all of the input data received from hardware. */
  @AutoLog
  public static class TransferIOInputs {

    public double transferWheelVelocity = 0.0;
    public double transferAppliedVolts = 0.0;
    public double ampsStator = 0.0;
    public double ampsSupply = 0.0;
    // public double[] TempCelcius = new double[] {};

    // constructor if needed for some inputs
    TransferIOInputs() {}
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(TransferIOInputs inputs) {}

  public default void setTransferController(double voltage) {}
}
