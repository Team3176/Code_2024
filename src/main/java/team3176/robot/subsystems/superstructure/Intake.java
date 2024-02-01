package team3176.robot.subsystems.superstructure;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import team3176.robot.constants.SuperStructureConstants;

public class Intake extends SubsystemBase {
  private static Intake instance;
  private final IntakeIO io;
  private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();
  private final DigitalInput linebreak1 = new DigitalInput(5);
  private final DigitalInput linebreak2 = new DigitalInput(6);

  private Intake(IntakeIO io) {
    this.io = io;
    SmartDashboard.putNumber("Intake_kp", SuperStructureConstants.INTAKE_kP);
    SmartDashboard.putNumber("Intake_Kg", SuperStructureConstants.INTAKE_kg);
    SmartDashboard.putNumber("Intake_angle", 0.0);
    SmartDashboard.putNumber("Intake_velocity", 0.0);
  }

  public void setCoastMode() {
    io.setCoastMode(true);
  }

  public void setBrakeMode() {
    io.setCoastMode(false);
  }

  public void setIntakeMotor(double velocity) {

    if (linebreak1.get() || linebreak2.get()) {
      System.out.println("Limitswitch value:" + linebreak1.get());
      io.setRoller(velocity);
    } else {
      io.stopRoller();
    }
  }

    public static Intake getInstance() {
        if (instance == null) {
          if (Constants.getMode() == Mode.REAL) {}
            instance = new Intake(new IntakeIOFalcon() {});
        } else {
          instance = new Intake(new IntakeIO() {});
        }
        return instance;
      }
    
}
