package team3176.robot.subsystems.superstructure;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;
import team3176.robot.Constants;
import team3176.robot.Constants.Mode;

public class Intake extends SubsystemBase {
  private static Intake instance;
  private final IntakeIO io;
  private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

  private Intake(IntakeIO io) {
    this.io = io;
  }

  public void setCoastMode() {
    io.setCoastMode(true);
  }

  public void setBrakeMode() {
    io.setCoastMode(false);
  }

  public Command runIntake(double percentoutput) {
    return this.runEnd(() -> set(percentoutput), () -> set(0.0));
  }

  public void set(double outputpercent) {
    io.set(outputpercent);
  }

  public static Intake getInstance() {
    if (instance == null) {
      if (Constants.getMode() != Mode.SIM) {
        instance = new Intake(new IntakeIOSpark() {});
      } else {
        instance = new Intake(new IntakeIOSim() {});
      }
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
