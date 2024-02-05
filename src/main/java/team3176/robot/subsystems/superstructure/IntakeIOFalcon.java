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
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.XboxController;
import team3176.robot.constants.Hardwaremap;

/** Template hardware interface for a closed loop subsystem. */
public class IntakeIOFalcon implements IntakeIO {

  TalonFX rollerController, pivotController;
  VelocityVoltage voltVelocity;
  PositionVoltage voltPosition;

  // DigitalInput limitswitch1;
  // DigitalInput linebreak2;
  NeutralOut brake;
  XboxController joystick;

  public IntakeIOFalcon() {

    TalonFXConfiguration rollerConfigs = new TalonFXConfiguration();
    TalonFXConfiguration pivotConfigs = new TalonFXConfiguration();
    brake = new NeutralOut();
    voltVelocity = new VelocityVoltage(0, 0, true, 0, 0, false, false, false);
    voltPosition = new PositionVoltage(0, 0, true, 0, 0, false, false, false);

    // limitswitch1 = new DigitalInput(Hardwaremap.intakeLimitswitch1_DIO);
    // linebreak2 = new DigitalInput(Hardwaremap.intakeLinebreak2_DIO);
    rollerController = new TalonFX(Hardwaremap.intakeRoller_CID);
    pivotController = new TalonFX(Hardwaremap.intakePivot_CID);

    // config setting
    rollerConfigs.Slot0.kP = 2.4; // An error of 0.5 rotations results in 1.2 volts output
    rollerConfigs.Slot0.kD = 0.1; // A change of 1 rotation per second results in 0.1 volts output
    rollerConfigs.Voltage.PeakForwardVoltage = 8;
    rollerConfigs.Voltage.PeakReverseVoltage = -8;

    rollerConfigs.Slot1.kP = 40; // An error of 1 rotations results in 40 amps output
    rollerConfigs.Slot1.kD = 2; // A change of 1 rotation per second results in 2 amps output
    // pivot configs

    pivotConfigs.Slot0.kP = 2.4; // An error of 0.5 rotations results in 1.2 volts output
    pivotConfigs.Slot0.kD = 0.1; // A change of 1 rotation per second results in 0.1 volts output
    // Peak output of 8 volts
    pivotConfigs.Voltage.PeakForwardVoltage = 8;
    pivotConfigs.Voltage.PeakReverseVoltage = -8;

    pivotConfigs.Slot1.kP = 40; // An error of 1 rotations results in 40 amps output
    pivotConfigs.Slot1.kD = 2; // A change of 1 rotation per second results in 2 amps output
    applyTalonFxConfigs(rollerController, rollerConfigs);
    applyTalonFxConfigs(pivotController, pivotConfigs);
  }
  /** Updates the set of loggable inputs. */
  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    // inputs.isLimitswitchOne = limitswitch1.get();
  }

  public void applyTalonFxConfigs(TalonFX controller, TalonFXConfiguration configs) {
    /* Retry config apply up to 5 times, report if failure */
    StatusCode status = StatusCode.StatusCodeNotInitialized;
    for (int i = 0; i < 5; ++i) {
      status = controller.getConfigurator().apply(configs);
      if (status.isOK()) break;
    }
    if (!status.isOK()) {
      System.out.println("Could not apply configs, error code: " + status.toString());
    }
  }

  public void stopRoller() {
    rollerController.setControl(brake);
  }

  public void stopPivot() {
    pivotController.setControl(brake);
  }

  @Override
  public void setRoller(double velocity) {
    rollerController.set(50);
  }

  @Override
  public void setPivot(double position) {
    pivotController.setControl(voltPosition.withPosition(position));

    // if(liitswitch1.get()  || limitswitch2.get()) {
    // System.out.println("ElevatorIOFalcon.set was called");
    // elevatorLeaderMotor.setControl(voltPosition.withPosition(.25));
    // elevatorLeaderMotor.set(1);
  }

  // public boolean getlinebreak1() {
  // return linebreak1.get();
  // }

  // public boolean getlinebreak2() {
  // return linebreak1.get();
  // }
}
