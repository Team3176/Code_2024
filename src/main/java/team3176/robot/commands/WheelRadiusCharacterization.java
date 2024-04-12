package team3176.robot.commands;

// Copyright (c) 2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.DoubleSupplier;
import lombok.RequiredArgsConstructor;
import org.littletonrobotics.junction.Logger;
import team3176.robot.subsystems.drivetrain.Drivetrain;
import team3176.robot.util.LoggedTunableNumber;

public class WheelRadiusCharacterization extends Command {
  private static final LoggedTunableNumber characterizationSpeed =
      new LoggedTunableNumber("WheelRadiusCharacterization/SpeedRadsPerSec", 0.7);
  private static final double driveRadius = Math.hypot(Drivetrain.LENGTH, Drivetrain.WIDTH);
  private static final DoubleSupplier gyroYawRadsSupplier =
      () -> Drivetrain.getInstance().getPoseOdom().getRotation().getRadians();

  @RequiredArgsConstructor
  public enum Direction {
    CLOCKWISE(-1),
    COUNTER_CLOCKWISE(1);

    private final int value;
  }

  private final Drivetrain drive;
  private final Direction omegaDirection;
  private final SlewRateLimiter omegaLimiter = new SlewRateLimiter(1.0);

  private double lastGyroYawRads = 0.0;
  private double accumGyroYawRads = 0.0;

  private double[] startWheelPositions;

  private double currentEffectiveWheelRadius = 0.0;

  public WheelRadiusCharacterization(Drivetrain drive, Direction omegaDirection) {
    this.drive = drive;
    this.omegaDirection = omegaDirection;
    addRequirements(drive);
  }

  @Override
  public void initialize() {
    // Reset
    lastGyroYawRads = gyroYawRadsSupplier.getAsDouble();
    accumGyroYawRads = 0.0;

    startWheelPositions = drive.getWheelRadiusCharacterizationPosition();

    omegaLimiter.reset(0);
  }

  @Override
  public void execute() {
    // Run drive at velocity
    drive.runWheelRadiusCharacterization(
        omegaLimiter.calculate(omegaDirection.value * characterizationSpeed.get()));

    // Get yaw and wheel positions
    accumGyroYawRads += MathUtil.angleModulus(gyroYawRadsSupplier.getAsDouble() - lastGyroYawRads);
    lastGyroYawRads = gyroYawRadsSupplier.getAsDouble();
    double averageWheelPosition = 0.0;
    double[] wheelPositiions = drive.getWheelRadiusCharacterizationPosition();
    for (int i = 0; i < 4; i++) {
      averageWheelPosition += Math.abs(wheelPositiions[i] - startWheelPositions[i]);
    }
    averageWheelPosition /= 4.0;

    currentEffectiveWheelRadius = (accumGyroYawRads * driveRadius) / averageWheelPosition;
    Logger.recordOutput("Drivetrain/RadiusCharacterization/DrivePosition", averageWheelPosition);
    Logger.recordOutput("Drivetrain/RadiusCharacterization/AccumGyroYawRads", accumGyroYawRads);
    Logger.recordOutput(
        "Drive/RadiusCharacterization/CurrentWheelRadiusInches",
        Units.metersToInches(currentEffectiveWheelRadius));
  }

  @Override
  public void end(boolean interrupted) {
    if (accumGyroYawRads <= Math.PI * 2.0) {
      System.out.println("Not enough data for characterization");
    } else {
      System.out.println(
          "Effective Wheel Radius: "
              + Units.metersToInches(currentEffectiveWheelRadius)
              + " inches");
    }
  }
}
