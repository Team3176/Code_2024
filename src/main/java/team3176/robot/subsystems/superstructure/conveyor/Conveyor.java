package team3176.robot.subsystems.superstructure.conveyor;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;
import team3176.robot.util.LoggedTunableNumber;

public class Conveyor extends SubsystemBase {
  private ConveyorIO io;

  private ConveyorIOInputsAutoLogged inputs;
  private LoggedTunableNumber conveyorVelocity;

  public Conveyor() {
    io = new ConveyorIOTalon();
    inputs = new ConveyorIOInputsAutoLogged();
    this.conveyorVelocity = new LoggedTunableNumber("conveyor/velocity", 0.6);
  }

  public Command run() {
    return this.runEnd(() -> io.setController(conveyorVelocity.get()), () -> io.setController(0));
  }

  public Command spit() {
    return this.runEnd(() -> io.setController(-1.5), () -> io.setController(0));
  }

  @Override
  public void periodic() {
    // TODO Auto-generated method stub
    io.updateInputs(inputs);
    Logger.processInputs("conveyor", inputs);
  }
}
