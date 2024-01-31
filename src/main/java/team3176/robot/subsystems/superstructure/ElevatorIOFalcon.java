// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team3176.robot.subsystems.superstructure;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.XboxController;
import team3176.robot.constants.SuperStructureConstants;
import team3176.robot.constants.Hardwaremap;
/** Template hardware interface for a closed loop subsystem. */
public class ElevatorIOFalcon implements ElevatorIO{
 
  TalonFX elevatorMotor; 
  PositionVoltage voltPosition;
  NeutralOut brake;
  DigitalInput limitswitch1;
  DigitalInput limitswitch2;

  public ElevatorIOFalcon() {
    TalonFXConfiguration configs = new TalonFXConfiguration();
    NeutralOut brake = new NeutralOut();
    PositionVoltage voltPosition = new PositionVoltage(0, 0, true, 0, 0, false, false, false);
    DigitalInput limitswitch1 = new DigitalInput(9);
    DigitalInput limitswitch2 = new DigitalInput(8);
    elevatorMotor = new TalonFX(Hardwaremap.elevator_CID);
    //config setting
    configs.Slot0.kP = 2.4; // An error of 0.5 rotations results in 1.2 volts output
    configs.Slot0.kD = 0.1; // A change of 1 rotation per second results in 0.1 volts output
    configs.Voltage.PeakForwardVoltage = 8;
    configs.Voltage.PeakReverseVoltage = -8;
    
    configs.Slot1.kP = 40; // An error of 1 rotations results in 40 amps output
    configs.Slot1.kD = 2; // A change of 1 rotation per second results in 2 amps output
       
    /* Retry config apply up to 5 times, report if failure */
    StatusCode status = StatusCode.StatusCodeNotInitialized;
    for (int i = 0; i < 5; ++i) {
      status = elevatorMotor.getConfigurator().apply(configs);
      System.out.println("Applied configs to: " + Hardwaremap.elevator_CID);
      if (status.isOK()) break;
    }
    if(!status.isOK()) {
      System.out.println("Could not apply configs, error code: " + status.toString());
    }    
  }
  /** Updates the set of loggable inputs. */
  @Override
  public void updateInputs(ElevatorIOInputs inputs) {
  }

  public void stop(){
    elevatorMotor.setControl(brake);
  }

  @Override
  public void set(double position) {
    System.out.println("ElevatorIOFalcon.set was called");
    elevatorMotor.setControl(voltPosition.withPosition(position));
    
 


  }

}

