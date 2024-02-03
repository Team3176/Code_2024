package team3176.robot.subsystems.superstructure;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Superstructure extends SubsystemBase {
  private static Superstructure instance;
  private Elevator elevator;
  private Intake intake;
  private Transfer transfer;
  private Shooter shooter;
  private static DigitalInput linebreak1;

  public Superstructure() {
    elevator = Elevator.getInstance();
    intake = Intake.getInstance();
    shooter = Shooter.getInstance();
    transfer = Transfer.getInstance();
  }

  public Command positiveIntake(double velocity) {
    return this.startEnd(() -> intake.setIntakeMotor(50), () -> intake.setStop());
  }

  public Command negativeIntake(double velocity) {
    return this.run(() -> intake.setIntakeMotor(-velocity));
  }

  public Command moveElevator(double position) {
    return this.run(() -> elevator.setElevatorMotor(.5));
  }

  public boolean linebreak1() {
    return linebreak1.get();
  }

  public static Superstructure getInstance() {
    if (instance == null) {
      instance = new Superstructure();
    }
    return instance;
  }

  /*
    public boolean getlinebreak1(){

      return linebreak1.get();
    }
  /* */
  public enum GamePiece {
    CUBE,
    CONE,
    NONE
  }
}
