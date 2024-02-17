package team3176.robot.subsystems.LED;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import team3176.robot.subsystems.LED.BlinkinLedDriver.BlinkinLedMode;

public class LEDSubsystem extends SubsystemBase {
  /** Creates a new LED. */
  private final BlinkinLedDriver blinkin;

  public LEDSubsystem() {
    blinkin = new BlinkinLedDriver(0);
  }

  @Override
  public void periodic() {}

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
