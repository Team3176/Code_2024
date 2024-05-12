package team3176.robot.subsystems.superstructure.intake;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.filter.LinearFilter;
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
import team3176.robot.util.LoggedTunableNumber;

public class Intake extends ProfiledPIDSubsystem {
  private static Intake instance;
  private IntakeIO io;
  private IntakeIOInputsAutoLogged inputs;

  private static final DCMotor motor = DCMotor.getFalcon500(1).withReduction(20);
  private static final SimpleMotorFeedforward motorFeedforward =
      new SimpleMotorFeedforward(0.0, 1.0 / motor.KvRadPerSecPerVolt);
  private static final double DEPLOY_POS = 2.1;

  private final LoggedTunableNumber rollerVolts;
  private double pivot_offset = 0;
  @AutoLogOutput private boolean ishomed = false;
  private double lastRollerSpeed = 0.0;
  private LinearFilter rollerAcceleration;

  private Intake(IntakeIO io) {
    super(new ProfiledPIDController(0.1, 0, 0, new Constraints(1.0, 1.0)));
    this.io = io;
    this.inputs = new IntakeIOInputsAutoLogged();
    this.rollerVolts = new LoggedTunableNumber("intake/rollerVolts", 7.0);
    rollerAcceleration = LinearFilter.movingAverage(5);
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

  /*
   * manual control
   */

  private Command disableControl() {
    return this.runOnce(this::disable);
  }

  public Command EmergencyHold() {
    return disableControl()
        .andThen(this.runEnd(() -> io.setPivotVolts(-2.5), () -> io.setPivotVolts(0.0)));
  }

  public Command manualDown() {
    return disableControl()
        .andThen(
            this.runEnd(
                () -> {
                  io.setPivotVolts(2.5);
                  io.setRollerVolts(4.0);
                },
                () -> {
                  io.setPivotVolts(0.0);
                  io.setRollerVolts(0);
                }));
  }

  private void runPivot(double volts) {
    // this assumes positive voltage deploys the intake and negative voltage retracts it.
    // invert the motor if that is NOT true
    io.setPivotVolts(volts);
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

  private void setGoalandEnable(double goal) {
    if (!this.isEnabled()) {
      this.enable();
    }
    this.setGoal(goal);
  }

  public Command retractPivot() {
    return this.runOnce(() -> this.setGoalandEnable(0.0));
  }

  public Command deployPivot() {
    return this.runOnce(() -> this.setGoalandEnable(DEPLOY_POS));
  }

  public Command climbIntake() {
    return this.runOnce(() -> this.setGoalandEnable(0.7));
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

  public Command spit() {
    return this.runEnd(() -> io.setRollerVolts(-1.5), () -> io.setRollerVolts(0));
  }

  @AutoLogOutput
  public double getRollerAcceleration() {
    return rollerAcceleration.calculate(inputs.rollerVelocityRadPerSec - lastRollerSpeed);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Intake", inputs);
    super.periodic();

    // Homing Logic
    if (inputs.lowerLimitSwitch && !ishomed) {
      ishomed = true;
      pivot_offset = inputs.pivotPosition - DEPLOY_POS;
    }

    lastRollerSpeed = inputs.rollerVelocityRadPerSec;
  }
}
