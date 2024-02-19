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
  public void periodic() {
    setBlinkinCode(-0.09);
  }

  public void noBallLight() {
    // off
    blinkin.setMode(BlinkinLedMode.SOLID_BLACK);
  }

  public void oneBallLight() {
    // Flashing Green
    blinkin.setMode(BlinkinLedMode.SOLID_RED);
  }

  public void twoBallLight() {
    // Solid Green
    blinkin.setMode(BlinkinLedMode.SOLID_BLUE);
  }
}
