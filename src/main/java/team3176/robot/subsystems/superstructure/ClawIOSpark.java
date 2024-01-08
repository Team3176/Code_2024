// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team3176.robot.subsystems.superstructure;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import team3176.robot.constants.Hardwaremap;
import edu.wpi.first.wpilibj.DigitalInput;

/** Template hardware interface for a closed loop subsystem. */
public class ClawIOSpark implements ClawIO{
  private CANSparkMax clawSpark;
  private DigitalInput linebreakOne;
  private DigitalInput linebreakTwo;
  /** Contains all of the input data received from hardware. */
  public ClawIOSpark() {
    clawSpark = new CANSparkMax(Hardwaremap.claw_CID, MotorType.kBrushless);
    linebreakOne = new DigitalInput(0);
    linebreakTwo = new DigitalInput(2);
  }


  /** Updates the set of loggable inputs. */
  @Override
  public void updateInputs(ClawIOInputs inputs) {
    inputs.isLinebreakOne = linebreakOne.get();
    inputs.isLinebreakTwo = linebreakTwo.get();
    inputs.velocity = clawSpark.getEncoder().getVelocity();
    inputs.appliedVolts = clawSpark.getAppliedOutput() * clawSpark.getBusVoltage();
    inputs.currentAmps = new double[]  {clawSpark.getOutputCurrent()};
    inputs.tempCelcius = new double[] {clawSpark.getMotorTemperature()};
  }

  
  /** Run open loop at the specified voltage. */
  @Override
  public void setClawMotor(double percent, int amps) {
    clawSpark.set(percent);
    clawSpark.setSmartCurrentLimit(amps);
  }
}
