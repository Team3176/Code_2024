// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team3176.robot.subsystems.superstructure;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

/** Template hardware interface for a closed loop subsystem. */
public interface ShooterIO {
  /** Contains all of the input data received from hardware. */
  @AutoLog
  public static class ShooterIOInputs {
    public Rotation2d pivotPosition = new Rotation2d();
    public double wheelPortVelocityRadPerSec = 0.0;
    public double wheelStarbrdVelocityRadPerSec = 0.0;
    public double pivotAppliedVolts = 0.0;
    public double wheelPortAppliedVolts = 0.0;
    public double wheelStarbrdAppliedVolts = 0.0;
    // public double[] CurrentAmps = new double[] {};
    // public double[] TempCelcius = new double[] {};

    // constructor if needed for some inputs
    ShooterIOInputs() {}
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(ShooterIOInputs inputs) {}

  public default void setWheelPortVoltage(double voltage) {}

  public default void setWheelStarbrdVoltage(double voltage) {}

  public default void setPivotVoltage(double voltage) {}

  public default void reset() {}
}
