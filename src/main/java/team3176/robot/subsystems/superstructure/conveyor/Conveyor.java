package team3176.robot.subsystems.superstructure.conveyor;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import team3176.robot.Constants;
import team3176.robot.Constants.Mode;
import team3176.robot.util.LoggedTunableNumber;

public class Conveyor extends SubsystemBase {
  private ConveyorIO io;
  private static Conveyor instance;

  private ConveyorIOInputsAutoLogged inputs;
  private LoggedTunableNumber conveyorIntakeFastVelocity;
  private LoggedTunableNumber conveyorIntakeSlowVelocity;
  private LoggedTunableNumber conveyorShootVelocity;

  private Conveyor(ConveyorIO io) {
    this.io = io;
    inputs = new ConveyorIOInputsAutoLogged();
    this.conveyorIntakeFastVelocity = new LoggedTunableNumber("conveyor/IntakeFast", 0.5);
    this.conveyorIntakeSlowVelocity = new LoggedTunableNumber("conveyor/IntakeSlow", 0.1);
    this.conveyorShootVelocity = new LoggedTunableNumber("conveyor/Shoot", 0.5);
  }

  @AutoLogOutput
  public boolean hasNote() {
    return isLaserIntakeSide() || isLaserShooterSide();
  }

  @AutoLogOutput
  public boolean NoteCentered() {
    return isLaserIntakeSide() && isLaserShooterSide();
  }

  public boolean hasNoteTooFar() {
    return !isLaserIntakeSide() && isLaserShooterSide();
  }

  public boolean isLaserIntakeSide() {
    return inputs.isFrontLinebreak;
    // return inputs.laserDistIntakeSide < 100;
  }

  public boolean isLaserIntakeSideFalse() {
    return !inputs.isFrontLinebreak;
    // return inputs.laserDistIntakeSide < 100;
  }

  public boolean isLaserShooterSide() {
    return inputs.isBackLinebreak;
    // return inputs.laserDistShooterSide < 100;
  }

  public Command runFast() {
    return this.runEnd(
        () -> io.setController(conveyorIntakeFastVelocity.get()), () -> io.setController(0));
  }

  public Command runSlow() {
    return this.runEnd(
        () -> io.setController(conveyorIntakeSlowVelocity.get()), () -> io.setController(0));
  }

  public Command runShoot() {
    return this.runEnd(
        () -> io.setController(conveyorShootVelocity.get()), () -> io.setController(0));
  }

  public Command spit() {
    return this.runEnd(() -> io.setController(-2.0), () -> io.setController(0));
  }

  public Command runSlowReverse() {
    return this.runEnd(() -> io.setController(-.1), () -> io.setController(0));
  }

  public Command centerNote() {
    return (this.runSlow()
            .until(() -> !isLaserIntakeSide())
            .andThen(runSlowReverse().until(() -> isLaserIntakeSide())))
        .withTimeout(2.0);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("conveyor", inputs);
  }

  public static Conveyor getInstance() {
    if (instance == null) {
      if (Constants.getMode() == Mode.REAL) {
        instance = new Conveyor(new ConveyorIOTalon());
      } else {
        instance = new Conveyor(new ConveyorIO() {});
      }
    }
    return instance;
  }
}
