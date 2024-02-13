package team3176.robot.subsystems.superstructure;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Superstructure extends SubsystemBase {
  private static Superstructure instance;
  private Elevator elevator;
  private Intake intake;
  // private Transfer transfer;
  private Shooter shooter;

  public Superstructure() {
    // elevator = Elevator.getInstance();
    // intake = Intake.getInstance();
    shooter = Shooter.getInstance();
    // transfer = Transfer.getInstance();
  }

  public Command positiveIntake(double velocity) {
    return this.run(() -> intake.setRollerVelocity(50))
        .until(() -> intake.getIsRollerLinebreak())
        .andThen(() -> intake.stopRoller());
  }

  public Command movePivotUp(double position) {
    return this.run(() -> intake.setPivotPosition(.25)).andThen(() -> intake.stopPivot());
  }

  public Command movePivotDown(double position) {
    return this.run(() -> intake.setPivotPosition(-position)).andThen(() -> intake.stopPivot());
  }

  public Command stopPivot() {
    return this.run(() -> intake.stopPivot());
  }

  public Command negativeIntake(double velocity) {
    return this.run(() -> intake.setRollerVelocity(-velocity));
  }

  public Command stopIntake() {
    return this.run(() -> intake.stopRoller());
  }

  public Command moveElevator(double position) {
    return this.run(() -> elevator.setElevatorMotor(.5));
  }

  public Command stopElevator() {
    return this.run(() -> elevator.stopElevator());
  }

  public Command shooterPercent(double percent) {
    return this.run(() -> shooter.setShooterPercent(percent));
  }

  public Command stopShooter() {
    return this.run(() -> shooter.setShooterStop());
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

}
