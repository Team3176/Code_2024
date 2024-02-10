package team3176.robot.subsystems.superstructure;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;
import team3176.robot.constants.*;
import team3176.robot.constants.RobotConstants.Mode;
import team3176.robot.util.TunablePID;

/** Elevator handles the height of the intake from the ground. */
public class Elevator extends SubsystemBase {
  private static Elevator instance;
  private final ElevatorIO io;
  private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();
  private TunablePID pid;

  private Elevator(ElevatorIO io) {
    this.io = io;
    pid = new TunablePID("Elevator/PID", 50.0, 0.0, 0.0);
  }

  public void setCoastMode() {
    io.setCoastMode(true);
  }

  public void setBrakeMode() {
    io.setCoastMode(false);
  }

  public void setElevatorMotor(double position) {

    // if(limitswitch1.get()  || limitswitch2.get()) {
    io.set(position);
  }

  public void stopElevator() {
    io.stop();
  }

  public double getPosition() {
    return inputs.Position;
  }

  public Command goToPosition(double position) {
    return this.runEnd(
        () -> {
          io.set(pid.calculate(getPosition(), position));
        },
        io::stop);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Elevator", inputs);
    pid.checkParemeterUpdate();
  }

  public static Elevator getInstance() {
    if (instance == null) {
      if (RobotConstants.getMode() == Mode.REAL) {
        instance = new Elevator(new ElevatorIOTalon() {});
      } else {
        instance = new Elevator(new ElevatorIOSim() {});
      }
    }
    return instance;
  }
}
