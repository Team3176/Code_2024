package team3176.robot.subsystems.superstructure.intake;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import org.littletonrobotics.junction.Logger;
import team3176.robot.Constants;
import team3176.robot.Constants.Mode;
import team3176.robot.constants.*;
import team3176.robot.util.LoggedTunableNumber;
import team3176.robot.util.TunablePID;

public class Intake extends SubsystemBase {
  private static Intake instance;
  private final IntakeIO io;
  private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();
  private final LoggedTunableNumber deployPivotVolts;
  private final LoggedTunableNumber rollerVolts;
  private final LoggedTunableNumber retractPivotVolts;
  private final LoggedTunableNumber waitTime;
  private final TunablePID pivotPID;
  private Timer deployTime = new Timer();
  private double pivotSetpoint;
  private final double DEPLOY_POS = 1.7;
  private double pivot_offset = 0;

  private enum pivotStates {
    DEPLOY,
    RETRACT,
    IDLE,
    HOLD
  };

  private pivotStates pivotState = pivotStates.HOLD;
  // DigitalInput linebreak1 = new DigitalInput(Hardwaremap.intakeRollerLinebreak_DIO);

  private Intake(IntakeIO io) {
    this.io = io;
    this.pivotPID = new TunablePID("intakePivot", 10.0, 0.5, 0.2);
    this.deployPivotVolts = new LoggedTunableNumber("intake/rollerDeployVolts", 0);
    this.rollerVolts = new LoggedTunableNumber("intake/rollerVolts", 4.0);
    this.retractPivotVolts = new LoggedTunableNumber("intake/rollerRetractVolts", 0);
    this.waitTime = new LoggedTunableNumber("intake/waitTime", 0);
  }

  private void stopRoller() {
    io.setRollerVolts(0.0);
  }

  private void pivotGoToPosition(int position) {
    io.setPivotPIDPosition(position);
  }

  private void stopPivot() {
    io.setPivotVolts(0.0);
  }

  private void runPivot(double volts) {
    // this assumes positive voltage deploys the intake and negative voltage retracks it.
    // invert the motor if that is NOT true
    if ((inputs.lowerLimitSwitch && volts > 0.0)) {
      volts = 0.0;
    } else if ((inputs.upperLimitSwitch && volts < 0.0)) {
      volts = -.2;
    }
    io.setPivotVolts(volts);
  }

  public static Intake getInstance() {
    if (instance == null) {
      if (Constants.getMode() == Mode.REAL) {
        instance = new Intake(new IntakeIOTalon() {});
      } else {
        instance = new Intake(new IntakeIOSim() {});
      }
    }
    return instance;
  }

  // Example command to show how to set the pivot state
  public Command deployPivot() {
    return this.runOnce(
        () -> {
          this.pivotState = pivotStates.DEPLOY;
          deployTime.restart();
        });
  }

  public Command retractPivot() {
    // TODO Implement
    return this.runOnce(() -> this.pivotState = pivotStates.RETRACT);
  }

  public Command spinIntakeUntilPivot() {
    // TODO
    // spin the intake until the first pivotlinebreak is triggered
    // do not set the intake to zero at the end that will be a seperate command
    // use the this.run() and a .until()
    return this.run(() -> io.setRollerVolts(rollerVolts.get()))
        .until(() -> inputs.isRollerLinebreak)
        .andThen(() -> io.setRollerVolts(0.0));
  }

  public Command spinIntake() {
    return this.runEnd(() -> io.setRollerVolts(rollerVolts.get()), () -> io.setRollerVolts(0));
  }

  public Command stopRollers() {
    // TODO
    // stop the rollers
    return this.runOnce(() -> io.setRollerVolts(0));
  }

  public Command intakeNote() {
    // the full deal.
    // deployPivot then spin the intake rollers until linebreak 1 then retract intake then stop
    // rollers.
    return (deployPivot()
            .andThen(spinIntakeUntilPivot())
            .andThen(retractPivot())
            .andThen(new WaitCommand(0.1))
            .andThen(stopRollers()))
        .finallyDo(
            () -> {
              pivotState = pivotStates.RETRACT;
              io.setRollerVolts(0.0);
            });
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Intake", inputs);
    Logger.recordOutput("Intake/state", pivotState);
    // double commandVolts = pivotPID.calculate(inputs.pivotPosition - pivot_offset, pivotSetpoint);
    // commandVolts = MathUtil.clamp(commandVolts, -8, 7.0);
    // Logger.recordOutput("Intake/PID_out", commandVolts);
    Logger.recordOutput("Intake/setpoint", this.pivotSetpoint);
    Logger.recordOutput("Intake/offsetPos", inputs.pivotPosition - pivot_offset);
    // runPivot(commandVolts);
    pivotPID.checkParemeterUpdate();
    if (inputs.upperLimitSwitch) {
      pivot_offset = inputs.pivotPosition;
    }
    // pivot state machine
    switch (pivotState) {
      case DEPLOY:
        if (deployTime.get() < 0.5) {
          runPivot(2.0);
        } else {
          runPivot(0);
        }
        if (inputs.lowerLimitSwitch) {
          pivotState = pivotStates.IDLE;
        }
        break;
      case RETRACT:
        runPivot(-2);
        if (inputs.upperLimitSwitch) {
          pivotState = pivotStates.HOLD;
        }
        break;
      case IDLE:
        runPivot(0.0);
        break;
      case HOLD:
        runPivot(-0.2);
        break;
    }
  }
}
