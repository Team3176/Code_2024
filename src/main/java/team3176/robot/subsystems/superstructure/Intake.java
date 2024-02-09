package team3176.robot.subsystems.superstructure;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;
import team3176.robot.constants.*;
import team3176.robot.constants.RobotConstants.Mode;

public class Intake extends SubsystemBase {
  private static Intake instance;
  private final IntakeIO io;
  private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();
  private int x = 0;
  // DigitalInput linebreak1 = new DigitalInput(Hardwaremap.intakeLinebreak1_DIO);

  private Intake(IntakeIO io) {
    this.io = io;
    SmartDashboard.putNumber("Intake_kp", SuperStructureConstants.INTAKE_PIVOT_kP);
    SmartDashboard.putNumber("Intake_Kg", SuperStructureConstants.INTAKE_PIVOT_kg);
    SmartDashboard.putNumber("Intake_angle", 0.0);
    SmartDashboard.putNumber("Intake_velocity", 0.0);
  }

  public void setCoastMode() {
    io.setCoastMode(true);
  }

  public void setBrakeMode() {
    io.setCoastMode(false);
  }

  public void setRollerVelocity(double velocity) {
    io.setRollerPercent(velocity);
  }

  public void setPivotPosition(double position) {
    io.setPivot(position);
  }

  public void stopRoller() {
    io.stopRoller();
  }

  public void stopPivot() {
    io.stopPivot();
  }

  public boolean getIsLinebreakOne() {
    // System.out.println("IsLimitswitchOne = " + inputs.isLimitswitchOne);
    return false; // (!linebreak1.get());
  }

  public static Intake getInstance() {
    if (instance == null) {
      if (RobotConstants.getMode() == Mode.REAL) {}
      instance = new Intake(new IntakeIOFalcon() {});
    } else {
      instance = new Intake(new IntakeIOSim() {});
    }
    return instance;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Intake", inputs);
    // Logger.recordOutput("Arm/mech2d", mech);
  }
}
