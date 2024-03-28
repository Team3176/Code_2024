package team3176.robot.subsystems.superstructure.transfer;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;
import team3176.robot.subsystems.superstructure.shooter.Shooter;
import team3176.robot.util.LoggedTunableNumber;

public class Transfer extends SubsystemBase {
  private TransferIOTalon io;

  private TransferIOInputsAutoLogged inputs;
  private LoggedTunableNumber transferVelocity;
  private InterpolatingDoubleTreeMap transferLookup;

  public Transfer() {
    io = new TransferIOTalon();
    inputs = new TransferIOInputsAutoLogged();
    this.transferVelocity = new LoggedTunableNumber("transfer/velocity", 0.5);
    this.transferLookup = new InterpolatingDoubleTreeMap();
    transferLookup.put(1.0, 0.6);
    transferLookup.put(3.2, 1.0);
  }

  public Command spinup() {
    return this.runEnd(
        () -> io.setTransferController(transferVelocity.get()), () -> io.setTransferController(0));
  }

  public Command shootLookup() {
    return this.runEnd(
        () -> io.setTransferController(transferLookup.get(Shooter.getInstance().getDistance())),
        () -> io.setTransferController(0));
  }

  public Command shoot(double velocity) {
    return this.runEnd(() -> io.setTransferController(velocity), () -> io.setTransferController(0));
  }

  public Command spit() {
    return this.runEnd(() -> io.setTransferController(-1.5), () -> io.setTransferController(0));
  }

  @Override
  public void periodic() {
    // TODO Auto-generated method stub
    io.updateInputs(inputs);
    Logger.processInputs("transfer", inputs);
  }
}
