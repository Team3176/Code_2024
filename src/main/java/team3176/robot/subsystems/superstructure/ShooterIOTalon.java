// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team3176.robot.subsystems.superstructure;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import team3176.robot.constants.Hardwaremap;

public class ShooterIOTalon implements ShooterIO {

  private TalonFX wheelUpperController =
      new TalonFX(Hardwaremap.shooterWheelUpper_CID, Hardwaremap.shooterWheelUpper_CBN);
  private TalonFX wheelLowerController =
      new TalonFX(Hardwaremap.shooterWheelLower_CID, Hardwaremap.shooterWheelLower_CBN);
  private TalonFX pivotController =
      new TalonFX(Hardwaremap.shooterPivot_CID, Hardwaremap.shooterPivot_CBN);
  private TalonFXConfiguration configsWheelUpper = new TalonFXConfiguration();
  private TalonFXConfiguration configsWheelLower = new TalonFXConfiguration();

  public ShooterIOTalon() {

    configsWheelUpper.Slot0.kP = 0.01; // An error of 1 rotation per second results in 2V output
    configsWheelUpper.Slot0.kI = 0.0; // An error of 1 rotation per second increases output by 0.5V every second
    configsWheelUpper.Slot0.kD = 0.0000; // A change of 1 rotation per second squared results in 0.01 volts output
    configsWheelUpper.Slot0.kV = 0.11; // Falcon 500 is a 500kV motor, 500rpm per V = 8.333 rps per V, 1/8.33 = 0.12 volts /
    // Rotation per second
    // Peak output of 8 volts
    configsWheelUpper.Voltage.PeakForwardVoltage = 8;
    configsWheelUpper.Voltage.PeakReverseVoltage = -8;

    configsWheelLower.Slot0.kP = 0.01; // An error of 1 rotation per second results in 2V output
    configsWheelLower.Slot0.kI = 0.0; // An error of 1 rotation per second increases output by 0.5V every second
    configsWheelLower.Slot0.kD = 0.0000; // A change of 1 rotation per second squared results in 0.01 volts output
    configsWheelLower.Slot0.kV = 0.11; // Falcon 500 is a 500kV motor, 500rpm per V = 8.333 rps per V, 1/8.33 = 0.12 volts /
    // Rotation per second
    // Peak output of 8 volts
    configsWheelLower.Voltage.PeakForwardVoltage = 8;
    configsWheelLower.Voltage.PeakReverseVoltage = -8;

    wheelUpperController.getConfigurator().apply(configsWheelUpper);
    wheelLowerController.getConfigurator().apply(configsWheelLower);
  }

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    // TODO: position is some gear math I don't know the gear sizes so do this later
    inputs.pivotPosition =
        new Rotation2d(); // Rotation2d.fromRotations(armEncoder.getAbsolutePosition().getValueAsDouble());
    inputs.wheelUpperVelocityRadPerSec =
        Units.rotationsToRadians(wheelUpperController.getVelocity().getValue());
    inputs.wheelLowerVelocityRadPerSec =
        Units.rotationsToRadians(wheelLowerController.getVelocity().getValue());
    /* TODO: below line needs to be updated to pheonix 6 calls for voltage measurement
    inputs.pivotAppliedVolts = pivotController.getAppliedOutput() * pivotController.getBusVoltage();
    */
    inputs.wheelUpperAppliedVolts = wheelUpperController.getMotorVoltage().getValue();
    inputs.wheelLowerAppliedVolts = wheelLowerController.getMotorVoltage().getValue();
  }

  public void applyTalonFxConfigs(TalonFX controller, TalonFXConfiguration configs) {
    /* Retry config apply up to 5 times, report if failure */
    StatusCode status = StatusCode.StatusCodeNotInitialized;
    for (int i = 0; i < 5; ++i) {
      status = controller.getConfigurator().apply(configs);
      System.out.println("Applied configs to: " + controller.getDeviceID());
      if (status.isOK()) break;
    }
    if (!status.isOK()) {
      System.out.println("Could not apply configs, error code: " + status.toString());
    }
  }

  @Override
  public void setPivotVoltage(double voltage) {
    pivotController.setVoltage(voltage);
  }

  @Override
  public void setPercentVoltage(double percent) {
    wheelLowerController.set(percent);
    wheelUpperController.set(percent);
  }

  @Override
  public void reset() {
    // to be implemented
  }
}
