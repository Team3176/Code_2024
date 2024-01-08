// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team3176.robot.subsystems.superstructure;

import org.littletonrobotics.junction.AutoLog;

/** Template hardware interface for a closed loop subsystem. */
public interface ArmIO{
  /** Contains all of the input data received from hardware. */
  @AutoLog
  public static class ArmIOInputs {
      public double Position = 0.0;
      public double VelocityRadPerSec = 0.0;
      public double AppliedVolts = 0.0;
      public double[] CurrentAmps = new double[] {};
      public double[] TempCelcius = new double[] {};

      //constructor if needed for some inputs
      ArmIOInputs() {
      }
  }
    
  /** Updates the set of loggable inputs. */
  public default void updateInputs(ArmIOInputs inputs) {}
  public default void set(double percentOutput){}
  public default void setCoastMode(boolean isCoastMode) {}
  public default void reset() {}
}

