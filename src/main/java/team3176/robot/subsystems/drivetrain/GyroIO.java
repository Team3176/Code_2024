// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team3176.robot.subsystems.drivetrain;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

/** Template hardware interface for a closed loop subsystem. */
public interface GyroIO {
  /** Contains all of the input data received from hardware. */
  public static class GyroIOInputs implements LoggableInputs {
    double pitch = 0.0;
    double yaw = 0.0;
    double roll = 0.0;
    Rotation2d rotation2d;

    GyroIOInputs() {
      rotation2d = new Rotation2d();
    }

    @Override
    public void toLog(LogTable table) {
      table.put("Pitch", pitch);
      table.put("Yaw", yaw);
      table.put("Roll", roll);
      table.put("rotation2d", rotation2d.getRadians());
    }

    @Override
    public void fromLog(LogTable table) {
      pitch = table.getDouble("Pitch", pitch);
      yaw = table.getDouble("Yaw", yaw);
      roll = table.getDouble("Roll", roll);
      rotation2d = Rotation2d.fromRadians(table.getDouble("rotation2d", rotation2d.getRadians()));
    }
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(GyroIOInputs inputs) {}

  public default void reset() {}
}
