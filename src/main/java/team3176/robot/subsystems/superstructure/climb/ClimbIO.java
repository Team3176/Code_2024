// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team3176.robot.subsystems.superstructure.climb;

import org.littletonrobotics.junction.AutoLog;

/** Template hardware interface for the Elevator subsystem. */
public interface ClimbIO {
  /** Contains all of the input data received from hardware. */
  @AutoLog
  public static class ClimbIOInputs {
    public double leftPosition = 0.0;
    public double rightPosition = 0.0;

    public double VelocityRadPerSec = 0.0;
    public double AppliedVolts = 0.0;
    public double[] CurrentAmps = new double[] {};
    public double[] TempCelcius = new double[] {};
    public boolean isLeftLimitswitch = true;
    public boolean isRightLimitswitch = true;

    // constructor if needed for some inputs
    ClimbIOInputs() {}
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(ClimbIOInputs inputs) {}

  public default void setLeft(double percentOutput) {}

  public default void setRight(double percentOutput) {}

  public default void setLeftPIDPosition(int encoderValue) {}

  public default void stopLeft() {}

  public default void stopRight() {}

  public default void getRightLimitswitch() {}

  public default void setCoastMode(boolean isCoastMode) {}

  public default void setclimbLBLimitswitchZero() {}

  public default void setclimbRBLimitswitchZero() {}

  public default void setLeftVoltage(int voltage) {}

  public default void setRightVoltage(int voltage) {}

  public default void reset() {}
}