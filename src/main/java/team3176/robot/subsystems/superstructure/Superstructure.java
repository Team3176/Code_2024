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
    System.out.println("Superstructure constructed #######################################");
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

  public Command upperShooterVelocity(double velocity) {
    return this.run(() -> shooter.setUpperShooterVelocityVoltage(velocity));
  }

  public Command lowerShooterVelocity(double velocity) {
    return this.run(() -> shooter.setLowerShooterVelocityVoltage(velocity));
  }

  public Command lowerShooterVelocity2(double velocity) {
    return this.run(() -> shooter.setLowerShooterVelocityVoltage2(velocity));
  }

  public Command runShooterPivot(double volts) {
    return shooter.pivotVoltage(volts);
  }
  // public Command shooterPivot(double percent) {
  //   return this.run(() -> shooter.setShooterPivotPercent(percent));
  // }

  // public Command stopShooterPivot() {
  //   return this.run(() -> shooter.setShooterPivotPercent(0));
  // }

  // public Command stopShooterPivotPID() {
  //   return this.run(() -> shooter.stopShooterPivotPosition());
  // }

  // public Command shooterPivotPID(int Position) {
  //   return this.run(() -> shooter.setShooterPivotPosition(Position));
  // }

  public Command stopShooter() {
    return this.run(() -> shooter.setShooterStop());
  }

  public static Superstructure getInstance() {
    if (instance == null) {
      instance = new Superstructure();
    }
    return instance;
  }

  public Command kms() {
    System.out.println("kms was calledf ##########################################");
    return this.run(() -> shooter.debugger());
  }

  /*
    public boolean getlinebreak1(){

      return linebreak1.get();
    }
  /* */

}
