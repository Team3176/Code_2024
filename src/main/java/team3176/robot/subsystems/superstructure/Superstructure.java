package team3176.robot.subsystems.superstructure;

import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.DoubleSupplier;
// import java.util.function.IntSupplier;
import team3176.robot.subsystems.drivetrain.Drivetrain;
import team3176.robot.subsystems.superstructure.climb.Climb;
import team3176.robot.subsystems.superstructure.elevator.Elevator;
import team3176.robot.subsystems.superstructure.intake.Intake;
import team3176.robot.subsystems.superstructure.shooter.Shooter;
import team3176.robot.subsystems.superstructure.transfer.Transfer;
import team3176.robot.util.NoteVisualizer;

public class Superstructure {
  private static Superstructure instance;
  private Climb climb;
  private Elevator elevator;
  private Intake intake;
  private Transfer transfer;
  private Shooter shooter;

  public Superstructure() {
    NoteVisualizer.setRobotPoseSupplier(Drivetrain.getInstance()::getPose);
    climb = Climb.getInstance();
    elevator = Elevator.getInstance();
    intake = Intake.getInstance();
    shooter = Shooter.getInstance();
    transfer = new Transfer();
  }

  public Command aimShooter() {
    return shooter.aim().alongWith(transfer.shoot());
  }

  public Command shoot() {
    return intake.spinIntake();
  }

  public Command runShooterPivot(double volts) {
    return shooter.pivotVoltage(volts);
  }

  public Command shooterPivotPID(int Position) {
    return shooter.pivotSetPositionOnce(Position);
  }

  public Command setClimbLeftPosition(DoubleSupplier position) {
    return climb.setLeftPosition((int) position.getAsDouble());
  }

  public Command setClimbRightPosition(DoubleSupplier position) {
    return climb.setRightPosition((int) position.getAsDouble());
  }

  public Command stopClimbLeft() {
    return climb.stopLeft();
  }

  public Command stopClimbRight() {
    return climb.stopRight();
  }

  public static Superstructure getInstance() {
    if (instance == null) {
      instance = new Superstructure();
      System.out.println("Superstructure instance created.");
    }
    return instance;
  }
}
