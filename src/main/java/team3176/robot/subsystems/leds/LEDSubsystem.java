package team3176.robot.subsystems.leds;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import team3176.robot.subsystems.leds.BlinkinLedDriver.BlinkinLedMode;

public class LEDSubsystem extends SubsystemBase {
  /** Creates a new LED. */
  private final BlinkinLedDriver blinkin;

  private static LEDSubsystem instance;

  private LEDSubsystem() {
    blinkin = new BlinkinLedDriver(9);
  }

  public static LEDSubsystem getInstance() {
    if (instance == null) {
      instance = new LEDSubsystem();
    }
    return instance;
  }

  public void setBlinkinCode(double blinkinCode) {
    blinkin.setCode(blinkinCode);
  }

  @Override
  public void periodic() {
    // if (DriverStation.getMatchTime() < 20 && DriverStation.getMatchTime() > 18) {
    //   isEndGame();
    // }
  }

  private void wantNote() {
    // Would prefer flashing orange
    blinkin.setMode(BlinkinLedMode.FIXED_STROBE_RED);
  }

  private void hasNote() {
    // Solid Orange
    blinkin.setMode(BlinkinLedMode.SOLID_ORANGE);
  }

  private void isAimedAtSpeaker() {
    // Solid Green
    blinkin.setMode(BlinkinLedMode.SOLID_GREEN);
  }

  private void isEndGame() {
    // Flashing Blue
    blinkin.setMode(BlinkinLedMode.FIXED_SHOT_BLUE);
  }

  private void isAuton() {
    // Rainbow
    blinkin.setMode(BlinkinLedMode.FIXED_RAINBOW_RAINBOW);
  }

  private void off() {
    blinkin.setMode(BlinkinLedMode.SOLID_BLACK);
  }

  // use SceduleCommand to not take on LED as a requirment and to not wait on it
  public Command setHasNote() {
    return new ScheduleCommand(this.runEnd(() -> hasNote(), () -> off()).withTimeout(1.0));
  }
  /*
   * run with Proxy
   */
  public Command AutoDriveStart() {
    return this.run(() -> isAuton());
  }

  public Command DefaultLED() {
    return this.run(() -> blinkin.setMode(BlinkinLedMode.SOLID_VIOLET));
  }
  public Command aiming(BooleanSupplier isOnTarget) {
    return this.run(() -> {
      if(isOnTarget.getAsBoolean()){
        blinkin.setMode(BlinkinLedMode.SOLID_YELLOW);
      } else {
        blinkin.setMode(BlinkinLedMode.SOLID_GREEN);
      }
    });
  }
}
