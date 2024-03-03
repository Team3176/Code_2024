package team3176.robot.subsystems.leds;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.BooleanSupplier;
import team3176.robot.constants.Hardwaremap;
import team3176.robot.subsystems.leds.BlinkinLedDriver.BlinkinLedMode;

public class LEDSubsystem extends SubsystemBase {
  /** Creates a new LED. */
  private final BlinkinLedDriver blinkin;

  private static LEDSubsystem instance;

  private LEDSubsystem() {
    blinkin = new BlinkinLedDriver(Hardwaremap.blinkin_pwm_port);
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

  /*
  private void isAimedAtSpeaker() {
    // Solid Green
    blinkin.setMode(BlinkinLedMode.SOLID_GREEN);
  }
  */

  private void isEndGame() {
    // Flashing Blue
    blinkin.setMode(BlinkinLedMode.FIXED_SHOT_BLUE);
  }

  private void isAuton() {
    // Rainbow
    blinkin.setMode(BlinkinLedMode.FIXED_RAINBOW_PARTY);
  }

  private void off() {
    blinkin.setMode(BlinkinLedMode.SOLID_BLACK);
  }

  public Command setOff() {
    return this.run(() -> off());
  }
  // use SceduleCommand to not take on LED as a requirment and to not wait on it
  public Command setHasNote() {
    return this.run(() -> blinkin.setMode(BlinkinLedMode.SOLID_GREEN));
  }
  /*
   * run with Proxy
   */
  public Command AutoDriveStart() {
    return this.run(() -> isAuton());
  }

  public Command EndgameStart() {
    return this.run(() -> isEndGame()).withTimeout(1);
  }

  public Command DefaultLED() {
    return this.run(() -> blinkin.setMode(BlinkinLedMode.SOLID_VIOLET));
  }

  public Command GreenFlash(double secs) {
    return this.run(() -> blinkin.setMode(BlinkinLedMode.SOLID_GREEN))
        .withTimeout(secs)
        .andThen(this.run(() -> off()).withTimeout(secs));
  }

  public Command GreenDoubleFlash() {
    return GreenFlash(0.35).andThen(GreenFlash(0.35)).andThen(GreenFlash(0.35));
  }

  public Command aiming(BooleanSupplier isOnTarget) {
    return this.run(
        () -> {
          if (isOnTarget.getAsBoolean()) {
            blinkin.setMode(BlinkinLedMode.SOLID_YELLOW);
          } else {
            blinkin.setMode(BlinkinLedMode.SOLID_GREEN);
          }
        });
  }
}
