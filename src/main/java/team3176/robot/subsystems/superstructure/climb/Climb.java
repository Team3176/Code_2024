package team3176.robot.subsystems.superstructure.climb;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;
import team3176.robot.Constants;
import team3176.robot.Constants.Mode;
import team3176.robot.constants.*;
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

  public Command stopLeft() {
    return this.runOnce(() -> io.setLeftVoltage(0.0));
  }

  public Command stopRight() {
    return this.runOnce(() -> io.setRightVoltage(0.0));
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
  private void leftGoToPosition(double position) {
    if (position > SuperStructureConstants.CLIMBLEFT_TOP_POS) {
      position = SuperStructureConstants.CLIMBLEFT_TOP_POS;
    } else if (position < 0.0) {
      position = 0.0;
    }
    io.setLeftPIDPosition(position);
  }

  private void rightGoToPosition(double position) {
    if (position > SuperStructureConstants.CLIMBRIGHT_TOP_POS) {
      position = SuperStructureConstants.CLIMBRIGHT_TOP_POS;
    } else if (position < 0.0) {
      position = 0.0;
    }
    io.setRightPIDPosition(position);
  }

  public Command setLeftPosition(DoubleSupplier position) {
    return this.runEnd(
        () -> {
          leftGoToPosition((position.getAsDouble()));
        },
        () -> io.setLeftVoltage(0.0));
  }

  public Command setRightPosition(DoubleSupplier position) {
    return this.runEnd(
        () -> {
          rightGoToPosition((position.getAsDouble()));
        },
        () -> io.setRightVoltage(0.0));
  }

  public Command moveRightPosition(DoubleSupplier delta) {
    return this.runEnd(
        () -> {
          rightGoToPosition(inputs.rightPosition + delta.getAsDouble());
        },
        () -> io.setRightVoltage(0.0));
  }

  public Command moveLeftPosition(DoubleSupplier delta) {
    return this.runEnd(
        () -> {
          leftGoToPosition(inputs.leftPosition + delta.getAsDouble());
        },
        () -> io.setLeftVoltage(0.0));
  }

  /*   public Command rightGoToPosition(double position) {
    return this.runEnd(
        () -> {
          io.setRight(pid.calculate(getRightPosition(), position));
        },
        io::stopRight);
  } */

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Climb", inputs);
    pid.checkParemeterUpdate();
  }

  public static Climb getInstance() {
    if (instance == null) {
      if (Constants.getMode() == Mode.REAL) {
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
