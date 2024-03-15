// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team3176.robot.subsystems.drivetrain;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

/** Template hardware interface for a closed loop subsystem. */
public interface GyroIO {
  /** Contains all of the input data received from hardware. */
  @AutoLog
  public static class GyroIOInputs {
    double pitch = 0.0;
    double yaw = 0.0;
    double roll = 0.0;
    Rotation2d rotation2d = new Rotation2d();
    boolean isConnected = false;
    public double[] odometryYawTimestamps = new double[] {};
    public Rotation2d[] odometryYawPositions = new Rotation2d[] {};
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(GyroIOInputs inputs) {}

  public default void reset() {}
}
