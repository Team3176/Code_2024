package team3176.robot.subsystems.superstructure;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Superstructure extends SubsystemBase {
  private static Superstructure instance;
  private Elevator elevator;
  private Intake intake;
  private Transfer transfer;
  private Shooter shooter;

  public Superstructure() {
    elevator = Elevator.getInstance();
    intake = Intake.getInstance();
    shooter = Shooter.getInstance();
    transfer = Transfer.getInstance();
  }

    public Command moveIntake(double velocity){
        return this.run(() -> intake.setIntakeMotor(velocity));
    }

    
  public static Superstructure getInstance() {
    if (instance == null) {
      instance = new Superstructure();
    }
    return instance;
  }

  public enum GamePiece {
    CUBE,
    CONE,
    NONE
  }
}
