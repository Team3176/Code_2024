/// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team3176.robot.subsystems.superstructure;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Command;

import team3176.robot.subsystems.superstructure.IntakeCubeIO.IntakeCubeIOInputs;
import org.littletonrobotics.junction.Logger;

import team3176.robot.constants.Hardwaremap;

public class IntakeCube extends SubsystemBase {
  /** Creates a new IntakeCube. */
  private TalonFX rollermotor = new TalonFX(Hardwaremap.intake_CID);
  private TalonSRX conveyor = new TalonSRX(61); //TODO: Add to HArdwareMap
  private DoubleSolenoid pistonOne;
  private DigitalInput linebreak;

  private static IntakeCube instance;
  private final IntakeCubeIO io;
  private final IntakeCubeIOInputs inputs = new IntakeCubeIOInputs();
  public IntakeCube(IntakeCubeIO io) 
  {
    this.io = io;
    pistonOne = new DoubleSolenoid(PneumaticsModuleType.REVPH, 4, 6);
    //pistonTwo = new DoubleSolenoid(PneumaticsModuleType.REVPH, 3, 2);
    linebreak = new DigitalInput(4);
  }

  public void spinIntake(double pct) {
    rollermotor.configPeakOutputReverse(pct);
    rollermotor.configPeakOutputForward(pct);
    rollermotor.set(ControlMode.PercentOutput, pct);
  }

  public void spinConveyor(double pct) {
    conveyor.configPeakOutputReverse(pct);
    conveyor.configPeakOutputForward(pct);
    conveyor.set(ControlMode.PercentOutput, pct);
  }

  public void setCoastMode() {
    rollermotor.setNeutralMode(NeutralMode.Coast);
  }

  public void setBrakeMode() {
    rollermotor.setNeutralMode(NeutralMode.Brake);
  } 

  public void Extend() {
    pistonOne.set(Value.kForward);
    //pistonTwo.set(Value.kForward);
  }

  public void Retract() {
    pistonOne.set(Value.kReverse);
    //pistonTwo.set(Value.kReverse);
  }

  public boolean getLinebreak()
  {
    return linebreak.get();
  }

  public static IntakeCube getInstance(){
    if ( instance == null ) {
      instance = new IntakeCube(new IntakeCubeIO() {});
    }
    return instance;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.getInstance().processInputs("IntakeCube", inputs);
    Logger.recordOutput("IntakeCube/Velocity", getVelocity());
    Logger.recordOutput("IntakeCube/Linebreak", getIsLinebreakLogger());
    Logger.recordOutput("IntakeCube/Extended", getIsExtended());
    // This method will be called once per scheduler run
    // Code stating if something is in the Intake
    SmartDashboard.putBoolean("CubeLinebreak", getLinebreak());
    // SmartDashboard.putBoolean("isInIntake", isInIntake);
    // SmartDashboard.putBoolean("isExtended", isExtended);

   }

   public double getVelocity()
   {
    return inputs.velocity;
   }

   public boolean getIsLinebreakLogger()
   {
    return inputs.isLinebreak;
   }

   public boolean getIsExtended()
   {
    return inputs.isextended;
   }

   public void runVoltage(double volts)
   {
    io.setVoltage(volts);
   }

   public void setVelocity(double velocity)
   {
    io.setVelocity(velocity);
   }
  public Command spit() {
    return this.runEnd(() -> {spinConveyor(0.6); spinIntake(1);},
     () -> {spinConveyor(0.0);spinIntake(0);});
  }
  public Command extendAndSpin() {
    return this.startEnd(() ->{
      this.Extend();
      this.spinIntake(.1);
    }, () -> {
      this.Retract();
      this.spinIntake(0.0);
    });
  }
  public Command retractSpinNot() {
    return this.runOnce(() -> {this.Retract(); this.spinIntake(0.0);});
  }
  
  public Command extendAndFreeSpin() {
    return this.startEnd(() ->{
      this.Extend();
      this.setCoastMode();
    }, () -> {
      this.Retract();
      this.setBrakeMode();
    });
  }

  public Command bumpConveyor() {
    return this.startEnd(() ->{
      this.spinConveyor(-.8);
    }, () -> {
      this.spinConveyor(0);
    }).withName("bumpConveyor");
  }
  public Command bumpConveyorTimeout(double timeout) {
    return this.startEnd(() ->{
      this.spinConveyor(-.8);
    }, () -> {
      this.spinConveyor(0);
    }).withTimeout(timeout).withName("bumpConveyor");
  }

}
