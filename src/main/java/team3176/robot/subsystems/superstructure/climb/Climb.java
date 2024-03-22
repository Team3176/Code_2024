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
  private double rightSetPoint = 0;
  // private double leftOffPoint;

  private final ClimbIOInputsAutoLogged inputs = new ClimbIOInputsAutoLogged();

  private TunablePID pid = new TunablePID("pid", 0.001, 0, 0);
  private TunablePID leftPIDController = new TunablePID("climbLeft", 1, 0, 0);
  private TunablePID rightPIDController = new TunablePID("climbRight", 1, 0, 0);

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

  public void leftPIDVoltageRoll() {
    double leftVoltage = leftPIDController.calculate(gyro.getRoll(), leftSetPoint);
    Logger.recordOutput("climb/roll", gyro.getRoll());
    Logger.recordOutput("climb/leftvoltage", leftVoltage);
    io.setLeftVoltage(leftVoltage);
    // return leftVoltage;
  }

  public void rightPIDVoltageRoll() {
    double rightVoltage = rightPIDController.calculate(gyro.getRoll(), rightSetPoint);
    Logger.recordOutput("climb/roll", gyro.getRoll());
    Logger.recordOutput("climb/rightvoltage", rightVoltage);
    io.setRightVoltage(rightVoltage);
    // return rightVoltage;
  }

  public void leftRightPIDVoltageRoll() {
    double rightVoltage = rightPIDController.calculate(gyro.getRoll(), rightSetPoint);
    Logger.recordOutput("climb/roll", gyro.getRoll());
    Logger.recordOutput("climb/rightvoltage", rightVoltage);
    io.setRightVoltage(rightVoltage);
    // return rightVoltage;
    double leftVoltage = leftPIDController.calculate(gyro.getRoll(), leftSetPoint);
    Logger.recordOutput("climb/roll", gyro.getRoll());
    Logger.recordOutput("climb/leftvoltage", leftVoltage);
    io.setLeftVoltage(leftVoltage);
    // return leftVoltage;
  }

  public Command setLeftPIDVoltageRoll() {
    return this.runEnd(
        () -> leftPIDVoltageRoll(), () -> io.setLeftVoltage(0)); // , () -> stopLeft());
  }

  public Command setRightPIDVoltageRoll() {
    return this.runEnd(
        () -> rightPIDVoltageRoll(), () -> io.setRightVoltage(0)); // , () -> stopLeft());
  }

  public Command setRightLeftPIDVoltageRoll() {
    return this.runEnd(() -> leftRightPIDVoltageRoll(), () -> io.setClimbVoltge(0));
  }

  /*  public Command setRightLeftPIDVoltageRoll() {
    return this.runEnd(() -> rightPIDVoltageRoll(), () -> io.setRightVoltage(0))
        .alongWith(this.runEnd(() -> leftPIDVoltageRoll(), () -> io.setRightVoltage(0)));
  } */

  public double getLeftPosition() {
    return inputs.leftPosition;
  }

  public double getRightPosition() {
    return inputs.rightPosition;
  }

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

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Climb", inputs);
    pid.checkParemeterUpdate();
    leftPIDController.checkParemeterUpdate();
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
