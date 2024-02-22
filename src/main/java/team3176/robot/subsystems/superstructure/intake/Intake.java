package team3176.robot.subsystems.superstructure.intake;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;
import team3176.robot.Constants;
import team3176.robot.Constants.Mode;
import team3176.robot.constants.*;

public class Intake extends SubsystemBase {
  private static Intake instance;
  private final IntakeIO io;
  private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();
  private int x = 0;
  // DigitalInput linebreak1 = new DigitalInput(Hardwaremap.intakeLinebreak1_DIO);

  private Intake(IntakeIO io) {
    this.io = io;
  }

  public void setCoastMode() {
    io.setCoastMode(true);
  }

  public void setBrakeMode() {
    io.setCoastMode(false);
  }

  public void setRollerVelocity(double velocity) {
    io.setRollerVolts(velocity * 12);
  }

  public void setPivotPosition(double position) {
    io.setPivotPIDPosition(position);
  }

  public void stopRoller() {
    io.setRollerVolts(0.0);
  }

  public void pivotGoToPosition(int position) {
    io.setPivotPIDPosition(position);
  }

  public void stopPivot() {
    io.setPivotVolts(0.0);
  }

  public boolean getIsRollerLinebreak() {
    // System.out.println("IsLimitswitchOne = " + inputs.isLimitswitchOne);
    return (inputs.isRollerLinebreak);
  }

  public static Intake getInstance() {
    if (instance == null) {
      if (Constants.getMode() == Mode.REAL) {}
      instance = new Intake(new IntakeIOTalon() {});
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
