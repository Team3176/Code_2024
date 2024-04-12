package team3176.robot.subsystems.superstructure;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.AutoLogOutput;
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

  @AutoLogOutput
  public boolean readyToShoot() {

    ChassisSpeeds speed = Drivetrain.getInstance().getCurrentChassisSpeed();
    double driveVelocity = Math.hypot(speed.vxMetersPerSecond, speed.vyMetersPerSecond);
    boolean drivetrainReady =
        driveVelocity < 0.25 && Drivetrain.getInstance().aimErrorDegrees() < 3.0;
    return shooter.readyToShoot() && drivetrainReady;
  }

  public Command aimShooterTune() {
    return shooter.aim().alongWith(transfer.spinup());
  }

  public Command aimAuto() {
    return aimShooterTune()
        .raceWith(
            new WaitCommand(2.0).until(() -> !conveyor.hasNote()).andThen(new WaitCommand(0.1)));
  }

  public Command aimShooter(double upper, double lower, double angle, double transferVel) {
    return shooter.aim(upper, lower, angle).alongWith(transfer.shoot(transferVel));
  }

  public Command aimShooterLookup() {
    return shooter.aimLookup().alongWith(transfer.shoot(0.6));
  }

  public Command aimClose() {
    return aimShooter(90, 40, 38, 0.5).withName("aimClose");
  }

  public Command aimAmp(boolean withDrive) {
    if (withDrive) {
      return Drivetrain.getInstance()
          .goToPoint((FieldConstants.ampFace))
          .asProxy()
          .alongWith(
              new WaitCommand(10.0)
                  .until(
                      () ->
                          Drivetrain.getInstance()
                                  .distanceToPoint(AllianceFlipUtil.apply(FieldConstants.ampFace))
                              < 2.0)
                  .andThen(aimShooterAmp().alongWith(climb.setAmpPosition())))
          .withName("aimAmpandDrive");
    } else {
      return aimShooterAmp().alongWith(climb.setAmpPosition()).withName("aimAmp");
    }
  }

  private Command aimShooterAmp() {
    return aimShooter(20, 20, 35, 0.35);
  }

  public Command aimPodium() {
    return aimShooter(80, 80, 7, 0.6).withName("aimPodium");
  }

  public Command aimPass() {
    return aimShooter(60, 40, 27, 0.6).withName("aimPass");
  }

  public Command shoot() {
    return conveyor.runShoot().withName("shoot");
  }

  public Command intakeNote() {
    return (conveyor.runFast().until(conveyor::isLaserShooterSide).andThen(conveyor.runSlow()))
        .alongWith(intake.intakeNote())
        .until(conveyor::hasNoteTooFar)
        .andThen(conveyor.centerNote());
  }

  public Command retractIntakePivot() {
    return intake.retractPivot();
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
    return intake.spit().alongWith(transfer.spit()).alongWith(conveyor.spit()).withName("spit");
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
