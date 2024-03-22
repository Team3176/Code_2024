package team3176.robot.subsystems.superstructure.intake;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import team3176.robot.Constants;
import team3176.robot.Constants.Mode;
import team3176.robot.Constants.RobotType;
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
  private InterpolatingDoubleTreeMap kG = new InterpolatingDoubleTreeMap();
  private boolean ishomed = false;
  private double lastRollerSpeed = 0.0;

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
    this.pivotPID = new TunablePID("intakePivot", 4.0, 0.0, 0.0);
    this.deployPivotVolts = new LoggedTunableNumber("intake/rollerDeployVolts", 0);
    this.rollerVolts = new LoggedTunableNumber("intake/rollerVolts", 3.5);
    this.retractPivotVolts = new LoggedTunableNumber("intake/rollerRetractVolts", 0);
    this.waitTime = new LoggedTunableNumber("intake/waitTime", 0);
    // kG.put(27.0, 0.6);
    // kG.put(,)
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

  public Command EmergencyHold() {
    return this.runEnd(() -> io.setPivotVolts(-2.0), () -> io.setPivotVolts(0.0));
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

  private boolean rollerSwitch() {
    return lastRollerSpeed - inputs.rollerVelocityRadPerSec > 15.0;
  }

  public static Intake getInstance() {
    if (instance == null) {
      if (Constants.getMode() == Mode.REAL && Constants.getRobot() != RobotType.ROBOT_DEFENSE) {
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
          this.pivotSetpoint = 1.7;
          deployTime.restart();
        });
  }

  public Command retractPivot() {
    return this.runOnce(() -> this.pivotSetpoint = 0.0);
  }

  public Command spinIntakeUntilPivot() {
    return this.run(() -> io.setRollerVolts(rollerVolts.get()))
        .andThen(() -> io.setRollerVolts(0.0));
  }

  public Command spinIntakeUntilRoller() {
    return this.run(() -> io.setRollerVolts(rollerVolts.get()))
        .until(() -> rollerSwitch())
        .andThen(() -> io.setRollerVolts(0.0));
  }

  public Command spinIntake() {
    return this.runEnd(() -> io.setRollerVolts(rollerVolts.get()), () -> io.setRollerVolts(0));
  }

  public Command spinIntakeRollersSlow() {
    return this.runEnd(() -> io.setRollerVolts(rollerVolts.get() / 2), () -> io.setRollerVolts(0));
  }

  public Command stopRollers() {
    return this.runOnce(() -> io.setRollerVolts(0));
  }

  public Command intakeNote() {
    return (deployPivot()
            .andThen(spinIntakeUntilPivot())
            .andThen(retractPivot()) //
            .andThen(stopRollers()))
        .finallyDo(
            () -> {
              pivotState = pivotStates.RETRACT;
              io.setRollerVolts(0.0);
            });
  }

  public Command intakeNoteroller() {
    return (deployPivot()
            .andThen(spinIntakeUntilPivot())
            .andThen(retractPivot()) //
            .andThen(stopRollers()))
        .finallyDo(
            () -> {
              pivotState = pivotStates.RETRACT;
              io.setRollerVolts(0.0);
            });
  }

  public Command spit() {
    return this.runEnd(() -> io.setRollerVolts(-1.5), () -> io.setRollerVolts(0));
  }

  @Override
  public void periodic() {
    // io.updateInputs(inputs);
    // Logger.processInputs("Intake", inputs);
    // Logger.recordOutput("Intake/state", pivotState);

    // double commandVolts = pivotPID.calculate(inputs.pivotPosition - pivot_offset, pivotSetpoint);
    // commandVolts = MathUtil.clamp(commandVolts, -3, 1.0);
    // if (pivotSetpoint < 0.1) {
    //   commandVolts -= 0.5;
    // }
    // Logger.recordOutput("Intake/PID_out", commandVolts);
    // Logger.recordOutput("Intake/setpoint", this.pivotSetpoint);
    // Logger.recordOutput("Intake/offsetPos", inputs.pivotPosition - pivot_offset);
    // runPivot(commandVolts);
    // pivotPID.checkParemeterUpdate();
    // if (inputs.upperLimitSwitch && !ishomed) {
    //   ishomed = true;
    //   pivot_offset = inputs.pivotPosition;
    // }
    // lastRollerSpeed = inputs.rollerVelocityRadPerSec;
    // pivot state machine
    // switch (pivotState) {
    //   case DEPLOY:
    //     /*         if (deployTime.get() < 0.5) { */
    //     runPivot(2.0);
    //     /*         } else {
    //       runPivot(0);
    //     } */
    //     if (inputs.lowerLimitSwitch) {
    //       pivotState = pivotStates.IDLE;
    //     }
    //     break;
    //   case RETRACT:
    //     runPivot(-2);
    //     if (inputs.upperLimitSwitch) {
    //       pivotState = pivotStates.HOLD;
    //     }
    //     break;
    //   case IDLE:
    //     runPivot(0.0);
    //     break;
    //   case HOLD:
    //     runPivot(-0.2);
    //     break;
    // }
  }
}
