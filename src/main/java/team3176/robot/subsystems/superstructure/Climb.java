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
  private TunablePID pid;

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

  public Command leftGoToPosition(double position) {
    return this.runEnd(
        () -> {
          io.setLeft(pid.calculate(getLeftPosition(), position));
        },
        io::stopLeft);
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
      } else {
        instance = new Climb(new ClimbIOSim() {});
      }
    }
    return instance;
  }
}
