/// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team3176.robot.subsystems.superstructure;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.Command;

import team3176.robot.subsystems.superstructure.IntakeConeIO.IntakeConeIOInputs;
import org.littletonrobotics.junction.Logger;

public class IntakeCone extends SubsystemBase {
  /** Creates a new IntakeCone. */
  private CANSparkMax rollermotor = new CANSparkMax(8, MotorType.kBrushless);
  private DoubleSolenoid pistonOne;
  private DigitalInput linebreak;

  private static IntakeCone instance;
  private Claw claw;
  private final IntakeConeIO io;
  private final IntakeConeIOInputs inputs = new IntakeConeIOInputs();
  public IntakeCone(IntakeConeIO io) 
  {
    this.io = io;
    pistonOne = new DoubleSolenoid(PneumaticsModuleType.REVPH, 5, 3);
    //pistonTwo = new DoubleSolenoid(PneumaticsModuleType.REVPH, 3, 2);
    linebreak = new DigitalInput(3);

    claw = Claw.getInstance();

  }

  public void spinVelocityPercent(double pct, int amps) {
    rollermotor.set(-pct);
    rollermotor.setSmartCurrentLimit(amps);
  }

  public void spit() 
  {
    //System.out.println("m_Claw.score()");
    spinVelocityPercent(-1, 20);
  }

  public void idle() {
    //System.out.println("m_Claw.idle()");
    spinVelocityPercent(0, 0);
}

  public void setCoastMode() {
    rollermotor.setIdleMode(IdleMode.kCoast);
  }

  public void setBrakeMode() {
    rollermotor.setIdleMode(IdleMode.kBrake);
  } 

  public void Extend() {
    pistonOne.set(Value.kForward);

  }

  public void Retract() {
    pistonOne.set(Value.kReverse);
  }

  public boolean getLinebreak()
  {
    return linebreak.get();
  }

  public static IntakeCone getInstance(){
    if ( instance == null ) {
      instance = new IntakeCone(new IntakeConeIO() {});
    }
    return instance;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.getInstance().processInputs("IntakeCone", inputs);
    Logger.recordOutput("IntakeCone/Velocity", getVelocity());
    Logger.recordOutput("IntakeCone/Linebreak", getIsLinebreakLogger());
    Logger.recordOutput("IntakeCone/Extended", getIsExtended());
    // This method will be called once per scheduler run
    // Code stating if something is in the Intake
    // SmartDashboard.putBoolean("isInIntake", isInIntake);
    SmartDashboard.putBoolean("ConeLinebreak", getLinebreak());
    // SmartDashboard.putBoolean("isExtended", isExtended);

   }

   public Command coneToClaw() {  
    return this.run(() ->  {spit();})
                .until(() -> this.claw.getIsLinebreakTwo() == false)
                .andThen(new WaitCommand(0.5))
                .andThen(this.runOnce(()->idle())).withTimeout(2.0).finallyDo((b)->idle()).withName("coneToClaw");
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

  public Command extendAndSpin() {
    return this.startEnd(() ->{
      this.Extend();
      this.spinVelocityPercent(.1, 25);
    }, () -> {
      this.Retract();
      this.spinVelocityPercent(0.0, 0);
    });
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
}
