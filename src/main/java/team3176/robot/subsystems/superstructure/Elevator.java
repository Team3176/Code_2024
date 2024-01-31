package team3176.robot.subsystems.superstructure;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import team3176.robot.constants.SuperStructureConstants;

/** Elevator handles the height of the intake from the ground. */
public class Elevator extends SubsystemBase {
  private static Elevator instance;
  private final ElevatorIO io;
  private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();
  private final DigitalInput limitswitch1 = new DigitalInput(3);
  private final DigitalInput limitswitch2 = new DigitalInput(4);

  private Elevator(ElevatorIO io) {
    this.io = io;
    SmartDashboard.putNumber("Elevator_kp", SuperStructureConstants.ELEVATOR_kP);
    SmartDashboard.putNumber("Elevator_Kg", SuperStructureConstants.ELEVATOR_kg);
    SmartDashboard.putNumber("elevator_height", 0.0);
  }

  public void setCoastMode() {
    io.setCoastMode(true);
  }

  public void setBrakeMode() {
    io.setCoastMode(false);
  }

  public void setElevatorMotor(double position) {

    if (limitswitch1.get() || limitswitch2.get()) {
      io.set(position);
    } else {
      io.stop();
    }
  }

  public Command moveElevator(double position) {
    return this.run(() -> setElevatorMotor(position));
  }

  @Override
  public void periodic() {}

  public static Elevator getInstance() {
    if (instance == null) {
      instance = new Elevator(new ElevatorIO() {});
    }
    return instance;
  }
}
