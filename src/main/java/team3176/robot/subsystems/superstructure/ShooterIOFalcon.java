// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team3176.robot.subsystems.superstructure;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.hardware.CANcoder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import team3176.robot.constants.Hardwaremap;
import team3176.robot.constants.SuperStructureConstants;
import team3176.robot.constants.Hardwaremap;
public class ShooterIOFalcon implements ShooterIO{
  
  private TalonFX wheelPortController = new TalonFX(Hardwaremap.shooterWheelPort_CID, Hardwaremap.shooter_CBN);
  private TalonFX wheelStarbrdController = new TalonFX(Hardwaremap.shooterWheelStarbrd_CID, Hardwaremap.shooter_CBN);
  private TalonFX pivotController = new TalonFX(Hardwaremap.shooterPivot_CID, Hardwaremap.shooter_CBN);


  public ShooterIOFalcon() {
    wheelPortController.
//    pivotController.
  }
  @Override
  public void updateInputs(ShooterIOInputs inputs) {
 //   inputs.pivotPosition = Rotation2d.fromRotations(armEncoder.getAbsolutePosition().getValueAsDouble());
//    inputs.wheelPortVelocityRadPerSec = Units.degreesToRadians(armEncoder.getVelocity().getValueAsDouble());
//    inputs.wheelStarbrdVelocityRadPerSec = Units.degreesToRadians(armEncoder.getVelocity().getValueAsDouble());
//    inputs.pivotAppliedVolts = pivotController.getAppliedOutput() * pivotController.getBusVoltage();
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
  public void reset() {
    // to be implemented
  }
}
