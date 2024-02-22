// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team3176.robot.subsystems.superstructure.intake;

import org.littletonrobotics.junction.AutoLog;

/** Template hardware interface for a closed loop subsystem. */
public interface IntakeIO {
  /** Contains all of the input data received from hardware. */
  @AutoLog
  public static class IntakeIOInputs {
    public double pivotPosition = 0.0;
    public double pivotVelocityRadPerSec = 0.0;
    public double pivotAppliedVolts = 0.0;
    public double[] pivotCurrentAmps = new double[] {};
    public double[] pivotTempCelcius = new double[] {};
    public double rollerVelocityRadPerSec = 0.0;
    public double rollerAppliedVolts = 0.0;
    public double[] rollerCurrentAmps = new double[] {};
    public double[] rollerTempCelcius = new double[] {};
    public boolean isRollerLinebreak = true;
    public boolean isLinebreakTwo = true;

    // constructor if needed for some inputs
    IntakeIOInputs() {}
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(IntakeIOInputs inputs) {}

  public default void setRollerVolts(double volts) {}

  public default void setRollerPIDVelocity(double rpm) {}

  public default void setPivotVolts(double volts) {}

  public default void setPivotPIDPosition(double position) {}

  public default void setCoastMode(boolean isCoastMode) {}
}
