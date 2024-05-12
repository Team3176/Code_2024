package team3176.robot.subsystems.superstructure.intake;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import team3176.robot.Constants;
import team3176.robot.Constants.Mode;
import team3176.robot.Constants.RobotType;
import team3176.robot.constants.*;
import team3176.robot.util.LoggedTunableNumber;

public class Intake extends ProfiledPIDSubsystem {
  private static Intake instance;
  private final IntakeIO io;
  private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();
  private final DCMotor motor = DCMotor.getFalcon500(1).withReduction(20);
  private final SimpleMotorFeedforward motorFeedforward =
      new SimpleMotorFeedforward(0.0, 1.0 / motor.KvRadPerSecPerVolt);
  private final LoggedTunableNumber rollerVolts;
  private final double DEPLOY_POS = 2.1;
  private double pivot_offset = 0;
  @AutoLogOutput private boolean ishomed = false;
  private double lastRollerSpeed = 0.0;

  // DigitalInput linebreak1 = new DigitalInput(Hardwaremap.intakeRollerLinebreak_DIO);

  private Intake(IntakeIO io) {
    super(new ProfiledPIDController(0.1, 0, 0, new Constraints(1.0, 1.0)));
    this.io = io;
    this.rollerVolts = new LoggedTunableNumber("intake/rollerVolts", 7.0);
    this.enable();
  }

  @Override
  protected double getMeasurement() {
    return inputs.pivotPosition - pivot_offset;
  }

  @Override
  protected void useOutput(double output, State setpoint) {
    Logger.recordOutput("Intake/Setpoint_position", setpoint.position);
    Logger.recordOutput("Intake/Setpoint_velocity", setpoint.velocity);
    double ffVolts = motorFeedforward.calculate(setpoint.velocity);
    Logger.recordOutput("Intake/ffvolts", ffVolts);
    if (!ishomed) {
      runPivot(ffVolts);
    } else {
      output = MathUtil.clamp(output, -3.5, 2.0);
      runPivot(output + ffVolts);
    }
  }

  public Command EmergencyHold() {
    return this.runEnd(() -> io.setPivotVolts(-2.5), () -> io.setPivotVolts(0.0));
  }

  public Command manualDown() {
    return this.runEnd(
        () -> {
          io.setPivotVolts(2.5);
          io.setRollerVolts(4.0);
        },
        () -> {
          io.setPivotVolts(0.0);
          io.setRollerVolts(0);
        });
  }

  private void runPivot(double volts) {
    // this assumes positive voltage deploys the intake and negative voltage retracts it.
    // invert the motor if that is NOT true
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

  public Command retractPivot() {
    return this.runOnce(() -> this.setGoal(0.0));
  }

  public Command deployPivot() {
    return this.runOnce(() -> this.setGoal(DEPLOY_POS));
  }

  public Command climbIntake() {
    return this.runOnce(() -> this.setGoal(0.0));
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
  /*
   * this can be much simpler than before just needs to spin the intake and retract when done.
   * keep the high level logic up in superstructure
   */
  public Command intakeNote() {
    return (deployPivot()
        .andThen(spinIntake())
        .finallyDo(
            () -> {
              setGoal(0.0);
              io.setRollerVolts(0.0);
            }));
  }

  // TODO: might need to deploy the intake during a spit but maybe not
  public Command spit() {
    return this.runEnd(() -> io.setRollerVolts(-1.5), () -> io.setRollerVolts(0));
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Intake", inputs);
    super.periodic();
    Logger.recordOutput("Intake/setpoint", this.getMeasurement());

    if (inputs.lowerLimitSwitch && !ishomed) {
      ishomed = true;
      pivot_offset = inputs.pivotPosition - DEPLOY_POS;
    }
    lastRollerSpeed = inputs.rollerVelocityRadPerSec;
  }
}
