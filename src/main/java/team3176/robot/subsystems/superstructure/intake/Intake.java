package team3176.robot.subsystems.superstructure.intake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import org.littletonrobotics.junction.Logger;
import team3176.robot.Constants;
import team3176.robot.Constants.Mode;
import team3176.robot.constants.*;
import team3176.robot.subsystems.superstructure.intake.IntakeIO.IntakeIOInputs;
import team3176.robot.util.LoggedTunableNumber;

public class Intake extends SubsystemBase {
  private static Intake instance;
  private final IntakeIO io;
  private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();
  private final LoggedTunableNumber deployPivotVolts;
  private final LoggedTunableNumber rollerVolts;
  private final LoggedTunableNumber retractPivotVolts;
  private final LoggedTunableNumber waitTime;

  private enum pivotStates {
    DEPLOY,
    RETRACT,
    IDLE
  };

  private pivotStates pivotState = pivotStates.IDLE;
  // DigitalInput linebreak1 = new DigitalInput(Hardwaremap.intakeRollerLinebreak_DIO);

  private Intake(IntakeIO io) {
    this.io = io;
    this.deployPivotVolts = new LoggedTunableNumber("intake/rollerDeployVolts", 0);
    this.rollerVolts = new LoggedTunableNumber("intake/pivotvVlts", 10.0);
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
    if ((inputs.upperLimitSwitch && volts < 0.0) || (inputs.lowerLimitSwitch && volts > 0.0)) {
      volts = 0.0;
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
    return this.runOnce(() -> this.pivotState = pivotStates.DEPLOY);
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
    return this.run(() -> io.setRollerVolts(0.1)).until(() -> inputs.isPivotLinebreak);
  }

  public Command stopRollers() {
    // TODO
    // stop the rollers
    return this.run(() -> io.setRollerVolts(0));
  }

  public Command intakeNote() {
    // the full deal.
    // deployPivot then spin the intake rollers until linebreak 1 then retract intake then stop
    // rollers.
    return deployPivot()
        .andThen(spinIntakeUntilPivot())
        .andThen(retractPivot())
        .andThen(new WaitCommand(0.1))
        .andThen(stopRollers());
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Intake", inputs);

    // pivot state machine
    switch (pivotState) {
      case DEPLOY:
        runPivot(3);
        if (inputs.lowerLimitSwitch) {
          pivotState = pivotStates.IDLE;
        }
        break;
      case RETRACT:
        runPivot(-3);
        if (inputs.upperLimitSwitch) {
          pivotState = pivotStates.IDLE;
        }
        break;
      case IDLE:
        runPivot(0.0);
        break;
    }
  }
}
