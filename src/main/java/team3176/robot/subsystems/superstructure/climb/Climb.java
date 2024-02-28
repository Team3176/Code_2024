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

  /*
  public void stopLeft() {
    io.stopLeft();
  }
  */

  public Command stopLeft() {
    return this.runOnce(io::stopLeft);
  }

  /*
  public void stopRight() {
    io.stopRight();
  }
  */

  public Command stopRight() {
    return this.runOnce(io::stopRight);
  }

  public double getLeftPosition() {
    return inputs.leftPosition;
  }

  public double getRightPosition() {
    return inputs.rightPosition;
  }

  public void setRightVoltage(int voltage) {
    io.setRightVoltage(voltage);
  }

  public void setLeftVoltage(int voltage) {
    io.setLeftVoltage(voltage);
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
  public void leftGoToPosition(int position) {
    io.setLeftPIDPosition(position);
    // io.setLeft(pid.calculate(getLeftPosition(), position));
    System.out.println("climb.leftGoToPosition = " + position);
    System.out.println("climb.getPosition = " + inputs.leftPosition);
  }

  public void rightGoToPosition(int position) {
    io.setRightPIDPosition(position);
    // io.setRight(pid.calculate(getRightPosition(), position));
    System.out.println("climb.rightGoToPosition = " + position);
    System.out.println("climb.getPosition = " + inputs.rightPosition);
  }

  public Command setLeftPosition(DoubleSupplier position) {
    return this.runEnd(
        () -> {
          io.setLeftPIDPosition((int) (position.getAsDouble() * 30));
        },
        io::stopRight);
  }

  public Command setRightPosition(DoubleSupplier position) {
    return this.runEnd(
        () -> {
          io.setRightPIDPosition((int) (position.getAsDouble() * 30));
        },
        io::stopRight);
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
    if (inputs.isLeftLimitswitch) {
      io.stopLeft();
      io.setclimbLBLimitswitchZero();
      // System.out.println("climb left bottom.getPosition = " + inputs.leftPosition);

      // need to add reset to climbLeft encoder to 0;
    }
    if (inputs.isRightLimitswitch) {
      io.stopRight();
      io.setclimbRBLimitswitchZero();
      // System.out.println("climb right bottom.getPosition = " + inputs.rightPosition);
      // need to add reset to climbRight encoder to 0;
    }

    if (inputs.leftPosition >= SuperStructureConstants.CLIMBLEFT_TOP_POS) {
      io.stopLeft();
    }

    if (inputs.leftPosition >= SuperStructureConstants.CLIMBRIGHT_TOP_POS) {
      io.stopRight();
    }
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
