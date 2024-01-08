// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team3176.robot.subsystems.superstructure;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

/** Template hardware interface for a closed loop subsystem. */
public interface ClawIO{
  /** Contains all of the input data received from hardware. */
  public static class ClawIOInputs implements LoggableInputs {
    public double velocity = 0.0;
    public double appliedVolts = 0.0;
    public boolean isLinebreakOne = true;
    public boolean isLinebreakTwo = true;
    public double[] currentAmps = new double[] {};
    public double[] tempCelcius = new double[] {};



    public void toLog(LogTable table) {
      table.put("VelocityOutputPercent", velocity);
      table.put("isLinebreakOne", isLinebreakOne);
      table.put("isLinebreakTwo", isLinebreakTwo);
      table.put("AppliedVolts", appliedVolts);
      table.put("CurrentAmps", currentAmps);
      table.put("TempCelcius", tempCelcius);
    }

    public void fromLog(LogTable table) {
      velocity = table.getDouble("VelocityOutputPercent", velocity);
      isLinebreakOne = table.getBoolean("isLinebreakOne", isLinebreakOne);
      isLinebreakTwo = table.getBoolean("isLinebreakTwo", isLinebreakTwo);
      appliedVolts = table.getDouble("AppliedVolts", appliedVolts);
      currentAmps = table.getDoubleArray("CurrentAmps", currentAmps);
      tempCelcius = table.getDoubleArray("TempCelcius", tempCelcius);
    }
  }



  /** Updates the set of loggable inputs. */
  public default void updateInputs(ClawIOInputs inputs) {}

  /** Run open loop at the specified voltage. */
  public default void setClawMotor(double percent, int amps) {}
}
