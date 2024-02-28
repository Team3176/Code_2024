package team3176.robot.subsystems.superstructure.transfer;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.signals.InvertedValue;

import team3176.robot.util.LoggedTunableNumber;

public class Transfer extends SubsystemBase {
  private TransferIOTalon io;

  private TransferIOInputsAutoLogged inputs;
  private LoggedTunableNumber transferVelocity;

  public Transfer() {
    io = new TransferIOTalon();
    inputs = new TransferIOInputsAutoLogged();
    this.transferVelocity = new LoggedTunableNumber("transfer/velocity", 0.0);
  }

  public Command shoot() {
    return this.runEnd(
        () -> io.setTransferController(transferVelocity.get()), () -> io.setTransferController(0));
  }

  public Command spit() {
    return this.runEnd(
        () -> io.setTransferController(-(transferVelocity.get())), () -> io.setTransferController(0));
  }

  @Override
  public void periodic() {
    // TODO Auto-generated method stub
    io.updateInputs(inputs);
    Logger.processInputs("transfer", inputs);
  }
}
