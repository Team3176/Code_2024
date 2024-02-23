package team3176.robot.subsystems.superstructure.transfer;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Transfer extends SubsystemBase {
  private TransferIOTalon io;

  private TransferIOInputsAutoLogged inputs;

  public Transfer() {
    io = new TransferIOTalon();
    inputs = new TransferIOInputsAutoLogged();
  }

  public Command shoot() {
    return this.runEnd(() -> io.setTransferController(0.8), () -> io.setTransferController(0));
  }

  @Override
  public void periodic() {
    // TODO Auto-generated method stub
    io.updateInputs(inputs);
    Logger.processInputs("transfer", inputs);
  }
}
