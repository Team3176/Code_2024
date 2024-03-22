// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team3176.robot.subsystems.superstructure.climb;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.DigitalInput;
import team3176.robot.constants.Hardwaremap;
import team3176.robot.constants.SuperStructureConstants;
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
  private final StatusSignal<Double> rightError;
  private final StatusSignal<Double> leftError;
  private final StatusSignal<Double> rightVolts;
  private final StatusSignal<Double> leftVolts;
  private final StatusSignal<Double> rightAmps;
  private final StatusSignal<Double> leftAmps;

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
    // configsLeft.Voltage.PeakForwardVoltage = 8;
    // configsLeft.Voltage.PeakReverseVoltage = -8;
    configsLeft.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    configsLeft.SoftwareLimitSwitch.ForwardSoftLimitThreshold =
        SuperStructureConstants.CLIMBLEFT_TOP_POS;
    configsLeft.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    configsLeft.SoftwareLimitSwitch.ReverseSoftLimitThreshold =
        SuperStructureConstants.CLIMBLEFT_ZERO_POS;
    configsLeft.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    configsRight.Slot0.kP = 2.4; // An error of 1 rotations results in 40 amps output
    configsRight.Slot0.kI = 0.0; // A change of 1 rotation per second results in 0.1 volts output
    configsRight.Slot0.kD = 0.0; // A change of 1 rotation per second results in 0.1 volts output
    configsRight.Slot0.kV = 0.0; // A change of 1 rotation per second results in 0.1 volts output
    // configsRight.Voltage.PeakForwardVoltage = 8;
    // configsRight.Voltage.PeakReverseVoltage = -8;
    configsRight.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    configsRight.SoftwareLimitSwitch.ForwardSoftLimitThreshold =
        SuperStructureConstants.CLIMBRIGHT_TOP_POS;
    configsRight.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    configsRight.SoftwareLimitSwitch.ReverseSoftLimitThreshold =
        SuperStructureConstants.CLIMBRIGHT_ZERO_POS;
    configsRight.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    TalonUtils.applyTalonFxConfigs(climbLeft, configsLeft);
    TalonUtils.applyTalonFxConfigs(climbRight, configsRight);
    climbLeft.setInverted(true);
    climbRight.setInverted(false);

    leftPosition = climbLeft.getPosition();
    rightPosition = climbRight.getPosition();
    rightError = climbRight.getClosedLoopError();
    leftError = climbLeft.getClosedLoopError();
    rightAmps = climbRight.getStatorCurrent();
    leftAmps = climbLeft.getStatorCurrent();
    rightVolts = climbRight.getMotorVoltage();
    leftVolts = climbLeft.getMotorVoltage();

    climbRight.setPosition(0);
    climbLeft.setPosition(0.0);
    BaseStatusSignal.setUpdateFrequencyForAll(
        50,
        leftPosition,
        rightPosition,
        rightError,
        leftError,
        rightAmps,
        leftAmps,
        rightVolts,
        leftVolts);
    climbLeft.optimizeBusUtilization();
    climbRight.optimizeBusUtilization();
  }
  /** Updates the set of loggable inputs. */
  @Override
  public void updateInputs(ClimbIOInputs inputs) {
    BaseStatusSignal.refreshAll(
        leftPosition,
        rightPosition,
        rightError,
        leftError,
        rightAmps,
        leftAmps,
        rightVolts,
        leftVolts);
    inputs.isLeftLimitswitch = (!climbLBLimitswitch.get());
    inputs.isRightLimitswitch = (!climbRBLimitswitch.get());
    inputs.leftPosition = leftPosition.getValueAsDouble();
    inputs.rightPosition = rightPosition.getValueAsDouble();
    inputs.leftError = leftError.getValue();
    inputs.rightError = rightError.getValue();
    inputs.rightAmpsStator = rightAmps.getValue();
    inputs.leftAmpsStator = leftAmps.getValue();
    inputs.rightVolts = rightVolts.getValue();
    inputs.leftVolts = leftVolts.getValue();
  }

  @Override
  public void setLeftPIDPosition(double position) {
    climbLeft.setControl(voltPosition.withPosition(position));
  }

  @Override
  public void setRightPIDPosition(double position) {
    climbRight.setControl(voltPosition.withPosition(position));
  }

  @Override
  public void setRight(double percent) {
    climbRight.set(percent);
  }

  @Override
  public void setLeft(double percent) {
    climbLeft.set(percent);
  }

  @Override
  public void setRightVoltage(double voltage) {
    climbRight.setVoltage(voltage);
  }

  @Override
  public void setLeftVoltage(double voltage) {
    climbLeft.setVoltage(voltage);
  }

  @Override
  public void setClimbVoltge(double voltage) {
    climbLeft.setVoltage(voltage);
    climbRight.setVoltage(voltage);
  }
  // System.out.println("ElevatorIOFalcon.set was called");
  // elevatorLeaderMotor.setControl(voltPosition.withPosition(.25));
  // elevatorLeaderMotor.set(1);

}
