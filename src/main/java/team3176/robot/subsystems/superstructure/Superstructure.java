package team3176.robot.subsystems.superstructure;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import team3176.robot.subsystems.drivetrain.Drivetrain;
import team3176.robot.subsystems.superstructure.elevator.Elevator;
import team3176.robot.subsystems.superstructure.intake.Intake;
import team3176.robot.subsystems.superstructure.shooter.Shooter;
import team3176.robot.util.NoteVisualizer;

public class Superstructure extends SubsystemBase {
  private static Superstructure instance;
  private Elevator elevator;
  private Intake intake;
  // private Transfer transfer;
  private Shooter shooter;

  public Superstructure() {
    NoteVisualizer.setRobotPoseSupplier(Drivetrain.getInstance()::getPose);
    elevator = Elevator.getInstance();
    intake = Intake.getInstance();
    shooter = Shooter.getInstance();
    // transfer = Transfer.getInstance();
  }

  public Command runShooterPivot(double volts) {
    return shooter.pivotVoltage(volts);
  }

  public Command shooterPivotPID(int Position) {
    return shooter.pivotSetPositionOnce(Position);
  }

  public static Superstructure getInstance() {
    if (instance == null) {
      instance = new Superstructure();
    }
    return instance;
  }
}
