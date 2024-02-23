// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team3176.robot.subsystems.superstructure.intake;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.CANSparkBase;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import team3176.robot.constants.Hardwaremap;
import team3176.robot.util.TalonUtils;

/** Template hardware interface for a closed loop subsystem. */
public class IntakeIOTalon implements IntakeIO {

  private TalonFX rollerController;
  private TalonFX pivotController;
  VelocityVoltage voltVelocity;
  VoltageOut rollerVolts = new VoltageOut(0.0);
  VoltageOut pivotVolts = new VoltageOut(0.0);
  PositionVoltage voltPosition;
  private SparkPIDController pivotPID;

  DigitalInput rollerLinebreak;
  DigitalInput pivotLinebreak;
  DigitalInput upperLimitSwitch;
  DigitalInput lowerLimitSwitch;

  public IntakeIOTalon() {

    TalonFXConfiguration rollerConfigs = new TalonFXConfiguration();
    TalonFXConfiguration pivotConfigs = new TalonFXConfiguration();

    // voltVelocity = new VelocityVoltage(0, 0, true, 0, 0, false, false, false);
    // voltPosition = new PositionVoltage(0, 0, true, 0, 0, false, false, false);

    rollerLinebreak = new DigitalInput(Hardwaremap.intakeRollerLinebreak_DIO);
    pivotLinebreak = new DigitalInput(Hardwaremap.intakePivotLinebreak_DIO);

    upperLimitSwitch = new DigitalInput(Hardwaremap.intakeUpperLimitSwitch_DIO);
    lowerLimitSwitch = new DigitalInput(Hardwaremap.intakeLowerLimitSwitch_DIO);

    rollerController = new TalonFX(Hardwaremap.intakeRoller_CID);
    pivotController = new TalonFX(Hardwaremap.intakePivot_CID);

    // config setting
    rollerConfigs.CurrentLimits.StatorCurrentLimit = 30;
    rollerConfigs.CurrentLimits.StatorCurrentLimitEnable = true;

    // pivot configs

    // pivotConfigs.Slot0.kP = 2.4; // An error of 0.5 rotations results in 1.2 volts output
    // pivotConfigs.Slot0.kD = 0.1; // A change of 1 rotation per second results in 0.1 volts output
    pivotConfigs.Voltage.PeakForwardVoltage = 4;
    pivotConfigs.Voltage.PeakReverseVoltage = -4;
    pivotConfigs.CurrentLimits.StatorCurrentLimit = 10;
    pivotConfigs.CurrentLimits.StatorCurrentLimitEnable = true;
    pivotConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    TalonUtils.applyTalonFxConfigs(rollerController, rollerConfigs);
    TalonUtils.applyTalonFxConfigs(pivotController, pivotConfigs);
  }
  /** Updates the set of loggable inputs. */
  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    inputs.isRollerLinebreak = (!rollerLinebreak.get());
    inputs.isPivotLinebreak = (!pivotLinebreak.get());

    inputs.upperLimitSwitch = upperLimitSwitch.get();
    inputs.lowerLimitSwitch = lowerLimitSwitch.get();

    inputs.pivotAppliedVolts = pivotController.getMotorVoltage().getValueAsDouble();
    inputs.pivotCurrentAmps = pivotController.getStatorCurrent().getValueAsDouble();
    inputs.pivotTempCelcius = pivotController.getDeviceTemp().getValueAsDouble();
    inputs.pivotPosition = pivotController.getPosition().getValueAsDouble();
    inputs.pivotVelocityRadPerSec =
        Units.rotationsToRadians(pivotController.getVelocity().getValueAsDouble());

    inputs.rollerAppliedVolts = rollerController.getMotorVoltage().getValueAsDouble();
    inputs.rollerCurrentAmps = rollerController.getStatorCurrent().getValueAsDouble();
    inputs.rollerTempCelcius = rollerController.getDeviceTemp().getValueAsDouble();
    inputs.rollerVelocityRadPerSec =
        Units.rotationsToRadians(rollerController.getVelocity().getValueAsDouble());
  }

  @Override
  public void setRollerVolts(double volts) {
    rollerController.setControl(rollerVolts.withOutput(volts));
  }

  @Override
  public void setPivotPIDPosition(double position) {
    pivotPID.setReference(position, CANSparkBase.ControlType.kPosition);
  }

  @Override
  public void setPivotVolts(double volts) {
    pivotController.setControl(pivotVolts.withOutput(volts));
  }
}
