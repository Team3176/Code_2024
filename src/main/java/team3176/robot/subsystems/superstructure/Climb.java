package team3176.robot.subsystems.superstructure;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;
import team3176.robot.constants.*;
import team3176.robot.constants.RobotConstants.Mode;
import team3176.robot.util.TunablePID;

/** Elevator handles the height of the intake from the ground. */
public class Climb extends SubsystemBase {
  private static Climb instance;
  private final ClimbIO io;
  private final ClimbIOInputsAutoLogged inputs = new ClimbIOInputsAutoLogged();
  private TunablePID pid = new TunablePID("climbLeft", 0.001, 0, 0);

  private Climb(ClimbIO io) {
    this.io = io;
  }

  public void setCoastMode() {
    io.setCoastMode(true);
  }

  public void setBrakeMode() {
    io.setCoastMode(false);
  }

  public void setLeftPercent(double percent) {
    io.setLeft(percent);
  }

  public void setRightPercent(double percent) {
    io.setRight(percent);
  }

  public void stopLeft() {
    io.stopLeft();
  }

  public void stopRight() {
    io.stopRight();
  }

  public double getLeftPosition() {
    return inputs.leftPosition;
  }

  public double getRightPosition() {
    return inputs.rightPosition;
  }
  /*
  public Command leftGoToPosition(double position) {
    return this.runEnd(
        () -> {
          io.setLeft(pid.calculate(getLeftPosition(), position));
          System.out.println(position);
        },
        io::stopLeft);
  }
  */
  public void leftGoToPosition(double position) {
    io.setLeft(position);
    // io.setLeft(pid.calculate(getLeftPosition(), position));
    System.out.println("climb.leftGoToPosition = " + position);
    System.out.println("climb.getPosition = " + inputs.leftPosition);
  }

  public Command rightGoToPosition(double position) {
    return this.runEnd(
        () -> {
          io.setRight(pid.calculate(getRightPosition(), position));
        },
        io::stopRight);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Climb", inputs);
    pid.checkParemeterUpdate();
    if (inputs.isLeftLimitswitch) {
      io.stopLeft();
      // need to add reset to climbLeft encoder to 0;
    }
    if (inputs.isRightLimitswitch) {
      io.stopRight();
      // need to add reset to climbRight encoder to 0;
    }
  }

  public static Climb getInstance() {
    if (instance == null) {
      if (RobotConstants.getMode() == Mode.REAL) {
        instance = new Climb(new ClimbIOTalon() {});
        System.out.println("Climb instance created for Mode.REAL");
      } else {
        instance = new Climb(new ClimbIOSim() {});
        System.out.println("Climb instance created for Mode.SIM");
      }
    }
    return instance;
  }
}
