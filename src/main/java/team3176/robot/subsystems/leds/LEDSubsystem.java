package team3176.robot.subsystems.leds;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import team3176.robot.subsystems.leds.BlinkinLedDriver.BlinkinLedMode;

public class LEDSubsystem extends SubsystemBase {
  /** Creates a new LED. */
  private final BlinkinLedDriver blinkin;

  public LEDSubsystem() {
    blinkin = new BlinkinLedDriver(9);
  }

  public void setBlinkinCode(double blinkinCode) {
    blinkin.setCode(blinkinCode);
  }

  @Override
  public void periodic() {}

  public void wantNote() {
    // Would prefer flashing orange
    blinkin.setMode(BlinkinLedMode.FIXED_STROBE_RED);
  }

  public void hasNote() {
    // Solid Orange
    blinkin.setMode(BlinkinLedMode.SOLID_ORANGE);
  }

  public void isAimedAtSource() {
    // Solid Green
    blinkin.setMode(BlinkinLedMode.SOLID_GREEN);
  }

  public void endGameStarted() {
    // Flashing Blue
    blinkin.setMode(BlinkinLedMode.FIXED_SHOT_BLUE);
  }

  public void isAuton() {
    // Rainbow
    blinkin.setMode(BlinkinLedMode.FIXED_RAINBOW_RAINBOW);
  }
}
