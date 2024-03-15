package team3176.robot.subsystems.superstructure.climb;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;
import team3176.robot.Constants;
import team3176.robot.Constants.Mode;
import team3176.robot.constants.*;
// import team3176.robot.subsystems.superstructure.ClimbIOInputsAutoLogged;
import team3176.robot.util.TunablePID;

/** Elevator handles the height of the intake from the ground. */
public class Climb extends SubsystemBase {
  private static Climb instance;
  private final ClimbIO io;
  private AHRS gyro = new AHRS(SPI.Port.kMXP);
  private double leftSetPoint = 0;
  private double leftOffPoint;

  private final ClimbIOInputsAutoLogged inputs = new ClimbIOInputsAutoLogged();
  private TunablePID pid = new TunablePID("climbLeft", 0.001, 0, 0);
  private TunablePID leftPIDController = new TunablePID("climbLeft", 0.001, 0, 0);

  int counter = 0;

  private Climb(ClimbIO io) {
    this.io = io;
  }

  public Command stopLeft() {
    return this.runOnce(() -> io.setLeftVoltage(0.0));
  }

  public Command stopRight() {
    return this.runOnce(() -> io.setRightVoltage(0.0));
  }

  public Command stopLeftRight() {
    return this.runOnce(
        () -> {
          io.setLeftVoltage(0.0);
          io.setRightVoltage(0.0);
        });
  }

  public double leftPIDPosition() {
    double leftVoltage = leftPIDController.calculate(gyro.getRoll(), leftSetPoint);
    Logger.recordOutput("climb/roll", gyro.getRoll());
    Logger.recordOutput("climb/leftvoltage", leftVoltage);
    /* System.out.println(
    gyro.getRoll() + " WE'RE GETTING RHW GYRO ROLL!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"); */
    io.setLeftVoltage(leftVoltage);
    return leftVoltage;
  }

  public Command setLeftPIDPosition() {

    return this.runEnd(() -> leftPIDPosition(), () -> stopLeft());
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
    }
    if (position < 0.0) {
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
          io.setRightVoltage((5 * delta.getAsDouble()));
        },
        () -> io.setRightVoltage(0.0));
  }

  public Command moveLeftPosition(DoubleSupplier delta) {
    return this.runEnd(
        () -> {
          io.setLeftVoltage((5 * delta.getAsDouble()));
        },
        () -> io.setLeftVoltage(0.0));
  }

  public Command moveLeftRightPosition(DoubleSupplier deltaLeft, DoubleSupplier deltaRight) {
    return this.runEnd(
        () -> {
          io.setRightVoltage(5 * deltaRight.getAsDouble());
          io.setLeftVoltage(5 * deltaLeft.getAsDouble());
        },
        () -> {
          io.setRightVoltage(0.0);
          io.setLeftVoltage(0.0);
        });
  }

  public Command moveLeftRightPIDPosition() {
    System.out.println("MOVELEFTRUGHTPIDPOSITION IS RUNNING!!!!!!!!!!!!!!!!!!");
    return this.runEnd(
        () -> {
          io.setLeftVoltage(5 * leftPIDPosition());
        },
        () -> {
          io.setLeftVoltage(0.0);
        });
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
    leftPIDController.checkParemeterUpdate();
    // int counter = 0;
    // if (counter == 20) {
    //   System.out.println(
    //       gyro.getRoll() + " WE'RE GETTING RHW GYRO ROLL!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
    //   counter = 0;
    // }
    // counter++;
    // if (Math.abs(gyro.getRoll()) > 1) {
    //   io.setLeftVoltage(.5);
    // } else {
    //   io.setLeftVoltage(0);
    // }
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
