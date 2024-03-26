package team3176.robot.subsystems.superstructure;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import java.util.function.DoubleSupplier;
import team3176.robot.FieldConstants;
// import java.util.function.IntSupplier;
import team3176.robot.subsystems.drivetrain.Drivetrain;
import team3176.robot.subsystems.superstructure.climb.Climb;
import team3176.robot.subsystems.superstructure.conveyor.Conveyor;
import team3176.robot.subsystems.superstructure.intake.Intake;
import team3176.robot.subsystems.superstructure.shooter.Shooter;
import team3176.robot.subsystems.superstructure.transfer.Transfer;
import team3176.robot.util.AllianceFlipUtil;
import team3176.robot.util.NoteVisualizer;

public class Superstructure {
  private static Superstructure instance;
  private Climb climb;
  private Intake intake;
  private Transfer transfer;
  private Shooter shooter;
  private Conveyor conveyor;

  public Superstructure() {
    NoteVisualizer.setRobotPoseSupplier(Drivetrain.getInstance()::getPose);
    climb = Climb.getInstance();
    intake = Intake.getInstance();
    shooter = Shooter.getInstance();
    conveyor = Conveyor.getInstance();
    transfer = new Transfer();
  }

  public Command aimShooterTune() {
    return shooter.aim().alongWith(transfer.shoot());
  }

  public Command aimShooter(double upper, double lower, double angle, double transferVel) {
    return shooter.aim(upper, lower, angle).alongWith(transfer.shoot(transferVel));
  }

  public Command aimClose() {
    return aimShooter(60, 60, 35, 0.6);
  }

  public Command aimAmp(boolean withDrive) {
    // return aimShooter(17, 17, 30, 0.35).alongWith(climb.setAmpLeftPosition(),
    // climb.setAmpRightPosition()).alongWith(Drivetrain.getInstance().goToPoint(FieldConstants.ampFaceCorner));
    if (withDrive) {
      return Drivetrain.getInstance()
          .goToPoint(AllianceFlipUtil.apply(FieldConstants.ampFace))
          .alongWith(aimAmpShooterClimb());
    } else {
      return aimAmpShooterClimb();
    }
  }

  private Command aimAmpShooterClimb() {
    //return aimShooter(17, 17, 30, 0.35).alongWith(climb.setAmpPosition());
    return aimShooterTune().alongWith(climb.setAmpPosition());
  }

  public Command aimPodium() {
    return aimShooter(80, 80, 7, 0.6);
  }

  public Command shoot() {
    return conveyor.runShoot();
  }

  public Command intakeNote() {
    return (conveyor.runFast().until(conveyor::isLaserShooterSide).andThen(conveyor.runSlow()))
        .alongWith(intake.intakeNote())
        .until(conveyor::hasNoteTooFar)
        .andThen(conveyor.centerNote());
  }

  public Command runShooterPivot(double volts) {
    return shooter.pivotVoltage(volts);
  }

  public Command shooterPivotPID(int Position) {
    return shooter.pivotSetPositionOnce(Position);
  }

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

  /* 
  public Command climbDown() {
    return climb.moveLeftRightPosition(0, 0);
  }
  */

  public Command stopClimbLeft() {
    return climb.stopLeft();
  }

  public Command stopClimbRight() {
    return climb.stopRight();
  }

  public Command stopClimbLeftRight() {
    return climb.stopLeftRight();
  }

  public Command spit() {
    return intake.spit().alongWith(transfer.spit()).alongWith(conveyor.spit());
  }

  public Command getSourceNoteAuto() {
    return Drivetrain.getInstance()
        .goToPoint(FieldConstants.sourePickup)
        // .andThen(Drivetrain.getInstance().chaseNote().raceWith(intakeNote()));
        .andThen(Drivetrain.getInstance().chaseNote());
  }

  public Command scoreNoteCenterAuto() {
    return Drivetrain.getInstance()
        .goToPoint(FieldConstants.CenterScore)
        .andThen(
            Drivetrain.getInstance()
                .driveAndAim(() -> 0, () -> 0)
                .raceWith(
                    aimClose().raceWith(new WaitCommand(0.3).andThen(shoot().withTimeout(0.5)))));
  }

  public Command doItAll() {
    return new ConditionalCommand(
        scoreNoteCenterAuto().andThen(getSourceNoteAuto()).repeatedly(),
        getSourceNoteAuto().andThen(scoreNoteCenterAuto()).repeatedly(),
        () -> true);
  }

  public static Superstructure getInstance() {
    if (instance == null) {
      instance = new Superstructure();
      System.out.println("Superstructure instance created.");
    }
    return instance;
  }
}
