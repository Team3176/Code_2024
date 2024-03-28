// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team3176.robot.subsystems.superstructure.intake;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
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

  private final StatusSignal<Double> pivotAppliedVolts;
  private final StatusSignal<Double> pivotCurrentAmpsStator;
  private final StatusSignal<Double> pivotCurrentAmpsSupply;
  private final StatusSignal<Double> pivotVelocity;
  private final StatusSignal<Double> pivotPosition;
  private final StatusSignal<Double> pivotTemp;

  private final StatusSignal<Double> rollerAppliedVolts;
  private final StatusSignal<Double> rollerCurrentAmpsStator;
  private final StatusSignal<Double> rollerCurrentAmpsSupply;
  private final StatusSignal<Double> rollerVelocity;
  private final StatusSignal<Double> rollerTemp;

  public IntakeIOTalon() {

    TalonFXConfiguration rollerConfigs = new TalonFXConfiguration();
    TalonFXConfiguration pivotConfigs = new TalonFXConfiguration();

    // voltVelocity = new VelocityVoltage(0, 0, true, 0, 0, false, false, false);
    // voltPosition = new PositionVoltage(0, 0, true, 0, 0, false, false, false);

    // rollerLinebreak = new DigitalInput(Hardwaremap.intakeRollerLinebreak_DIO);
    // pivotLinebreak = new DigitalInput(Hardwaremap.intakePivotLinebreak_DIO);

    upperLimitSwitch = new DigitalInput(Hardwaremap.intakeUpperLimitSwitch_DIO);
    lowerLimitSwitch = new DigitalInput(Hardwaremap.intakeLowerLimitSwitch_DIO);

    rollerController = new TalonFX(Hardwaremap.intakeRoller_CID, Hardwaremap.intakeRoller_CBN);
    pivotController = new TalonFX(Hardwaremap.intakePivot_CID, Hardwaremap.intakePivot_CBN);

    // config setting
    // rollerConfigs.CurrentLimits.StatorCurrentLimit = 50;
    // rollerConfigs.CurrentLimits.StatorCurrentLimitEnable = true;

    // pivot configs

    // pivotConfigs.Slot0.kP = 2.4; // An error of 0.5 rotations results in 1.2 volts output
    // pivotConfigs.Slot0.kD = 0.1; // A change of 1 rotation per second results in 0.1 volts output
    pivotConfigs.Voltage.PeakForwardVoltage = 8;
    pivotConfigs.Voltage.PeakReverseVoltage = -10;
    pivotConfigs.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    pivotConfigs.Feedback.SensorToMechanismRatio = 20.0;

    pivotConfigs.CurrentLimits.SupplyCurrentLimit = 60;
    pivotConfigs.CurrentLimits.SupplyCurrentLimitEnable = true;
    pivotConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    TalonUtils.applyTalonFxConfigs(rollerController, rollerConfigs);
    TalonUtils.applyTalonFxConfigs(pivotController, pivotConfigs);
    pivotController.setPosition(0, 0);

    pivotAppliedVolts = pivotController.getMotorVoltage();
    pivotCurrentAmpsStator = pivotController.getStatorCurrent();
    pivotCurrentAmpsSupply = pivotController.getSupplyCurrent();
    pivotVelocity = pivotController.getVelocity();
    pivotPosition = pivotController.getPosition();
    pivotTemp = pivotController.getDeviceTemp();

    rollerAppliedVolts = rollerController.getMotorVoltage();
    rollerCurrentAmpsStator = rollerController.getStatorCurrent();
    rollerCurrentAmpsSupply = rollerController.getSupplyCurrent();
    rollerVelocity = rollerController.getVelocity();
    rollerTemp = rollerController.getDeviceTemp();

    BaseStatusSignal.setUpdateFrequencyForAll(
        50,
        pivotAppliedVolts,
        pivotCurrentAmpsStator,
        pivotVelocity,
        pivotPosition,
        pivotTemp,
        pivotCurrentAmpsSupply);
    BaseStatusSignal.setUpdateFrequencyForAll(
        50,
        rollerAppliedVolts,
        rollerVelocity,
        rollerCurrentAmpsStator,
        rollerTemp,
        rollerCurrentAmpsSupply);

    rollerController.optimizeBusUtilization();
    pivotController.optimizeBusUtilization();
  }
  /** Updates the set of loggable inputs. */
  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    BaseStatusSignal.refreshAll(
        pivotAppliedVolts,
        pivotCurrentAmpsStator,
        pivotVelocity,
        pivotPosition,
        pivotTemp,
        pivotCurrentAmpsSupply);
    BaseStatusSignal.refreshAll(
        rollerAppliedVolts,
        rollerVelocity,
        rollerCurrentAmpsStator,
        rollerTemp,
        rollerCurrentAmpsSupply);

    // inputs.isRollerLinebreak = (!rollerLinebreak.get());
    // inputs.isPivotLinebreak = (!pivotLinebreak.get());

    inputs.upperLimitSwitch = (!upperLimitSwitch.get());
    inputs.lowerLimitSwitch = !lowerLimitSwitch.get();

    inputs.pivotAppliedVolts = pivotAppliedVolts.getValueAsDouble();
    inputs.pivotAmpsStator = pivotCurrentAmpsStator.getValueAsDouble();
    inputs.pivotAmpsSupply = pivotCurrentAmpsSupply.getValue();
    inputs.pivotTempCelcius = pivotTemp.getValueAsDouble();
    inputs.pivotPosition = Units.rotationsToRadians(pivotPosition.getValueAsDouble());
    inputs.pivotVelocityRadPerSec = Units.rotationsToRadians(pivotVelocity.getValueAsDouble());

    inputs.rollerAppliedVolts = rollerAppliedVolts.getValueAsDouble();
    inputs.rollerAmpsStator = rollerCurrentAmpsStator.getValueAsDouble();
    inputs.rollerAmpsSupply = rollerCurrentAmpsSupply.getValueAsDouble();
    inputs.rollerTempCelcius = rollerTemp.getValueAsDouble();
    inputs.rollerVelocityRadPerSec = Units.rotationsToRadians(rollerVelocity.getValueAsDouble());
  }

  /*   @Override
  public Boolean getRollerLinebreak() {
    return
  } */

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
