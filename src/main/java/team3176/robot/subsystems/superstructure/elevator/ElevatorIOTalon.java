// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team3176.robot.subsystems.superstructure.elevator;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.DigitalInput;
import team3176.robot.constants.Hardwaremap;

/** Template hardware interface for the Elevator subsystem. */
public class ElevatorIOTalon implements ElevatorIO {

  TalonFX elevatorLeaderMotor, elevatorFollowerMotor;
  PositionVoltage voltPosition;
  NeutralOut brake;
  DigitalInput limitswitch1;
  DigitalInput limitswitch2;

  public ElevatorIOTalon() {
    TalonFXConfiguration configs = new TalonFXConfiguration();
    brake = new NeutralOut();
    voltPosition = new PositionVoltage(0, 0, true, 0, 0, false, false, false);
    limitswitch1 = new DigitalInput(Hardwaremap.elevatorLeaderLimitSwitch_DIO);
    limitswitch2 = new DigitalInput(Hardwaremap.elevatorFollowerLimitSwitch_DIO);
    elevatorLeaderMotor =
        new TalonFX(Hardwaremap.elevatorLeader_CID, Hardwaremap.elevatorLeaderMotor_CBN);
    elevatorFollowerMotor =
        new TalonFX(Hardwaremap.elevatorFollower_CID, Hardwaremap.elevatorFollowerMotor_CBN);
    // config setting
    configs.Slot0.kP = 2.4; // An error of 0.5 rotations results in 1.2 volts output
    configs.Slot0.kD = 0.1; // A change of 1 rotation per second results in 0.1 volts output
    configs.Voltage.PeakForwardVoltage = 8;
    configs.Voltage.PeakReverseVoltage = -8;

    configs.Slot1.kP = 40; // An error of 1 rotations results in 40 amps output
    configs.Slot1.kD = 2; // A change of 1 rotation per second results in 2 amps output
    applyTalonFxConfigs(elevatorLeaderMotor, configs);
    applyTalonFxConfigs(elevatorFollowerMotor, configs);
  }
  /** Updates the set of loggable inputs. */
  @Override
  public void updateInputs(ElevatorIOInputs inputs) {}

  public void applyTalonFxConfigs(TalonFX controller, TalonFXConfiguration configs) {
    /* Retry config apply up to 5 times, report if failure */
    StatusCode status = StatusCode.StatusCodeNotInitialized;
    for (int i = 0; i < 5; ++i) {
      status = elevatorLeaderMotor.getConfigurator().apply(configs);
      System.out.println("Applied configs to: " + Hardwaremap.elevatorLeader_CID);
      if (status.isOK()) break;
    }
    if (!status.isOK()) {
      System.out.println("Could not apply configs, error code: " + status.toString());
    }
  }

  public void stop() {
    elevatorLeaderMotor.setControl(brake);
  }

  public void setElevatorPIDPosition(int position) {
    elevatorLeaderMotor.setControl(voltPosition.withPosition(position));
  }

  @Override
  public void set(double position) {
    // if(limitswitch1.get()  || limitswitch2.get()) {
    if (limitswitch1.get()) {
      System.out.println("Limitswitch value:" + limitswitch1.get());
      elevatorLeaderMotor.set(.5);
    } else {
      stop();
    }
    // System.out.println("ElevatorIOFalcon.set was called");
    // elevatorLeaderMotor.setControl(voltPosition.withPosition(.25));
    // elevatorLeaderMotor.set(1);
  }
}
