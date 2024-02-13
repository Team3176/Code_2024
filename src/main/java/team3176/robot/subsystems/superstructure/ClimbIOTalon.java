// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team3176.robot.subsystems.superstructure;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.DigitalInput;
import team3176.robot.constants.Hardwaremap;

/** Template hardware interface for the Elevator subsystem. */
public class ClimbIOTalon implements ClimbIO {

  TalonFX climbLeft, climbRight;
  PositionVoltage voltPosition;
  NeutralOut brake;
  DigitalInput climbLBLimitswitch, climbRBLimitswitch;

  public ClimbIOTalon() {
    TalonFXConfiguration configsLeft = new TalonFXConfiguration();
    TalonFXConfiguration configsRight = new TalonFXConfiguration();
    brake = new NeutralOut();
    voltPosition = new PositionVoltage(0, 0, true, 0, 0, false, false, false);
    climbLBLimitswitch = new DigitalInput(Hardwaremap.climbLBLimitSwitch_DIO);
    climbRBLimitswitch = new DigitalInput(Hardwaremap.climbRBLimitSwitch_DIO);
    climbLeft = new TalonFX(Hardwaremap.climbLeft_CID, Hardwaremap.climbLeft_CBN);
    climbRight = new TalonFX(Hardwaremap.climbRight_CID, Hardwaremap.climbRight_CBN);
    // config setting
    configsLeft.Slot0.kP = 0.001; // An error of 0.5 rotations results in 1.2 volts output
    configsLeft.Slot0.kD = 0.1; // A change of 1 rotation per second results in 0.1 volts output
    configsLeft.Voltage.PeakForwardVoltage = 8;
    configsLeft.Voltage.PeakReverseVoltage = -8;
    climbLeft.setInverted(false);

    configsRight.Slot0.kP = 0.001; // An error of 1 rotations results in 40 amps output
    configsRight.Slot0.kD = 2; // A change of 1 rotation per second results in 2 amps output
    configsRight.Voltage.PeakForwardVoltage = 8;
    configsRight.Voltage.PeakReverseVoltage = -8;
    climbRight.setInverted(true);

    applyTalonFxConfigs(climbLeft, configsLeft);
    applyTalonFxConfigs(climbRight, configsRight);
  }
  /** Updates the set of loggable inputs. */
  @Override
  public void updateInputs(ClimbIOInputs inputs) {
    inputs.isLeftLimitswitch = (!climbLBLimitswitch.get());
    inputs.isRightLimitswitch = (!climbRBLimitswitch.get());
    inputs.leftPosition = climbLeft.getPosition().getValueAsDouble();
    inputs.rightPosition = climbRight.getPosition().getValueAsDouble();
  }

  public void applyTalonFxConfigs(TalonFX controller, TalonFXConfiguration configs) {
    /* Retry config apply up to 5 times, report if failure */
    StatusCode status = StatusCode.StatusCodeNotInitialized;
    for (int i = 0; i < 5; ++i) {
      status = controller.getConfigurator().apply(configs);
      System.out.println("Applied configs to: " + Hardwaremap.climbLeft_CID);
      if (status.isOK()) break;
    }
    if (!status.isOK()) {
      System.out.println("Could not apply configs, error code: " + status.toString());
    }
  }

  public void stopLeft() {
    climbLeft.setControl(brake);
  }

  public void stopRight() {
    climbRight.setControl(brake);
  }

  public double getLeftPosition() {
    return climbLeft.getPosition().getValueAsDouble();
  }

  public double getRightPosition() {
    return climbRight.getPosition().getValueAsDouble();
  }

  public boolean getLeftLimitswitch() {
    return (!climbLBLimitswitch.get());
  }

  public boolean getLeftRightswitch() {
    return (!climbRBLimitswitch.get());
  }

  @Override
  public void setLeft(double percent) {
    climbLeft.set(percent);
  }

  @Override
  public void setRight(double percent) {
    climbRight.set(percent);
  }
  // System.out.println("ElevatorIOFalcon.set was called");
  // elevatorLeaderMotor.setControl(voltPosition.withPosition(.25));
  // elevatorLeaderMotor.set(1);

}
