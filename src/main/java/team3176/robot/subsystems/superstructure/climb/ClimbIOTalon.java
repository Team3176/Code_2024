// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team3176.robot.subsystems.superstructure.climb;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.DigitalInput;
import team3176.robot.constants.Hardwaremap;
import team3176.robot.util.TalonUtils;

/** Template hardware interface for the Elevator subsystem. */
public class ClimbIOTalon implements ClimbIO {

  TalonFX climbLeft, climbRight;
  PositionVoltage voltPosition;
  NeutralOut brake;
  DigitalInput climbLBLimitswitch, climbRBLimitswitch;
  TalonFXConfiguration configsLeft, configsRight;
  private final StatusSignal<Double> rightPosition;
  private final StatusSignal<Double> leftPosition;

  public ClimbIOTalon() {
    configsLeft = new TalonFXConfiguration();
    configsRight = new TalonFXConfiguration();
    brake = new NeutralOut();
    voltPosition = new PositionVoltage(0, 0, true, 0, 0, false, false, false);
    climbLBLimitswitch = new DigitalInput(Hardwaremap.climbLBLimitSwitch_DIO);
    climbRBLimitswitch = new DigitalInput(Hardwaremap.climbRBLimitSwitch_DIO);
    climbLeft = new TalonFX(Hardwaremap.climbLeft_CID, Hardwaremap.climbLeft_CBN);
    climbRight = new TalonFX(Hardwaremap.climbRight_CID, Hardwaremap.climbRight_CBN);
    // config setting
    configsLeft.Slot0.kP = 2.4; // An error of 0.5 rotations results in 1.2 volts output
    configsLeft.Slot0.kI = 0.0; // A change of 1 rotation per second results in 0.1 volts output
    configsLeft.Slot0.kD = 0.1; // A change of 1 rotation per second results in 0.1 volts output
    configsLeft.Slot0.kV = 0.0; // A change of 1 rotation per second results in 0.1 volts output
    configsLeft.Voltage.PeakForwardVoltage = 4;
    configsLeft.Voltage.PeakReverseVoltage = -4;
    configsLeft.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    climbLeft.setInverted(false);

    configsRight.Slot0.kP = 2.4; // An error of 1 rotations results in 40 amps output
    configsRight.Slot0.kI = 0.0; // A change of 1 rotation per second results in 0.1 volts output
    configsRight.Slot0.kD = 0.1; // A change of 1 rotation per second results in 0.1 volts output
    configsRight.Slot0.kV = 0.0; // A change of 1 rotation per second results in 0.1 volts output
    configsRight.Voltage.PeakForwardVoltage = 4;
    configsRight.Voltage.PeakReverseVoltage = -4;
    climbRight.setInverted(true);
    configsRight.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    TalonUtils.applyTalonFxConfigs(climbLeft, configsLeft);
    TalonUtils.applyTalonFxConfigs(climbRight, configsRight);

    leftPosition = climbLeft.getPosition();
    rightPosition = climbRight.getPosition();

    climbRight.setPosition(0);
    climbLeft.setPosition(0.0);
    BaseStatusSignal.setUpdateFrequencyForAll(50, leftPosition,rightPosition);
    climbLeft.optimizeBusUtilization();
    climbRight.optimizeBusUtilization();

  }
  /** Updates the set of loggable inputs. */
  @Override
  public void updateInputs(ClimbIOInputs inputs) {
    BaseStatusSignal.refreshAll(leftPosition,rightPosition);
    inputs.isLeftLimitswitch = (!climbLBLimitswitch.get());
    inputs.isRightLimitswitch = (!climbRBLimitswitch.get());
    inputs.leftPosition = leftPosition.getValueAsDouble();
    inputs.rightPosition = rightPosition.getValueAsDouble();
  }

  @Override
  public void setLeft(double percent) {
    climbLeft.set(percent);
  }

  @Override
  public void setLeftPIDPosition(int position) {
    climbLeft.setControl(voltPosition.withPosition(position));
  }

  @Override
  public void setRightPIDPosition(int position) {
    climbRight.setControl(voltPosition.withPosition(position));
  }

  @Override
  public void setRight(double percent) {
    climbRight.set(percent);
  }

  @Override
  public void setRightVoltage(int voltage) {
    climbRight.setVoltage(voltage);
  }

  @Override
  public void setLeftVoltage(int voltage) {
    climbLeft.setVoltage(voltage);
  }
  // System.out.println("ElevatorIOFalcon.set was called");
  // elevatorLeaderMotor.setControl(voltPosition.withPosition(.25));
  // elevatorLeaderMotor.set(1);

}
