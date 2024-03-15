package team3176.robot.subsystems.superstructure;

import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.DoubleSupplier;
// import java.util.function.IntSupplier;
import team3176.robot.subsystems.superstructure.climb.Climb;

// TODO: uncomment all intake commands

public class Superstructure {
  private static Superstructure instance;
  private Climb climb;
  // private Elevator elevator;
  // private Intake intake;
  // private Transfer transfer;
  // private Shooter shooter;

  public Superstructure() {
    // NoteVisualizer.setRobotPoseSupplier(Drivetrain.getInstance()::getPose);
    climb = Climb.getInstance();
    // elevator = Elevator.getInstance();
    // intake = Intake.getInstance();
    // shooter = Shooter.getInstance();
    // transfer = new Transfer();
  }

  // public Command aimShooterTune() {
  //  return shooter.aim().alongWith(transfer.shoot());
  // }

  // public Command aimShooter(double upper, double lower, double angle, double transferVel) {
  //  return shooter.aim(upper, lower, angle).alongWith(transfer.shoot(transferVel));
  // }

  // public Command aimClose() {
  //  return aimShooter(60, 60, 30, 0.6);
  // }

  // public Command aimAmp() {
  //  return aimShooter(17, 17, 30, 0.35);
  // }

  // public Command aimPodium() {
  //  return aimShooter(80, 80, 7, 0.6);
  // }

  // public Command shoot() {
  //  return shooter.aim();
  //  // return intake.spinIntake();
  // }

  // public Command runShooterPivot(double volts) {
  //  return shooter.pivotVoltage(volts);
  // }

  // public Command shooterPivotPID(int Position) {
  //  return shooter.pivotSetPositionOnce(Position);
  // }

  public Command setClimbLeftPosition(DoubleSupplier position) {
    return climb.setLeftPosition(position);
  }

  public Command setClimbRightPosition(DoubleSupplier position) {
    return climb.setRightPosition(position);
  }

  public Command moveClimbLeftPosition(DoubleSupplier position) {
    return climb.moveLeftPosition(position);
  }

  public Command moveClimbRightPosition(DoubleSupplier position) {
    return climb.moveRightPosition(position);
  }

  public Command moveClimbLeftRightPosition(DoubleSupplier deltaLeft, DoubleSupplier deltaRight) {
    return climb.moveLeftRightPosition(deltaLeft, deltaRight);
  }

  public Command moveClimbLeftPIDPosition() {
    // return climb.moveLeftRightPIDPosition();
<<<<<<< HEAD
    return climb.setLeftPIDPosition().withName("climbPIDS");
=======
    return climb.setLeftPIDPosition();
    // .withName("climbPID");
>>>>>>> 9d17c3d7fbbd5040b30302e9d9758a8f588ae2e5
  }

  public Command stopClimbLeft() {
    return climb.stopLeft();
  }

  public Command stopClimbRight() {
    return climb.stopRight();
  }

  public Command stopClimbLeftRight() {
    return climb.stopLeftRight();
  }

  // public Command spit() {
  //  return shooter.aim();

  //  // return intake.spit().alongWith(transfer.spit());

  // }

  // public Command getSourceNoteAuto() {
  //  /* return Drivetrain.getInstance()
  //  .goToPoint(FieldConstants.sourePickup)
  //  .andThen(Drivetrain.getInstance().chaseNote().raceWith(intake.intakeNote())); */
  //  return shooter.aim();
  // }

  // public Command scoreNoteCenterAuto() {
  //  return Drivetrain.getInstance()
  //      .goToPoint(FieldConstants.CenterScore)
  //      .andThen(
  //          Drivetrain.getInstance()
  //              .driveAndAim(() -> 0, () -> 0)
  //              .raceWith(
  //                  aimClose().raceWith(new WaitCommand(0.3).andThen(shoot().withTimeout(0.5)))));
  // }

  // public Command doItAll() {
  //  /*     return new ConditionalCommand(
  //  scoreNoteCenterAuto().andThen(getSourceNoteAuto()).repeatedly(),
  //  getSourceNoteAuto().andThen(scoreNoteCenterAuto()).repeatedly(),);
  //          intake::hasNote); */
  //  return shooter.aim();
  // }

  public static Superstructure getInstance() {
    if (instance == null) {
      instance = new Superstructure();
      System.out.println("Superstructure instance created.");
    }
    return instance;
  }
}
