package team3176.robot.util;

import edu.wpi.first.wpilibj.Timer;
import java.util.function.BooleanSupplier;

public class DelayedBoolean {
  private BooleanSupplier trigger;
  private double delayTime;
  Timer timer = new Timer();
  boolean hasTriggered = false;
  /**
   * return true after a set delay of calling get for the first time
   *
   * @param seconds
   */
  public DelayedBoolean(double seconds) {
    delayTime = seconds;
    trigger = () -> true;
  }
  /**
   * return true after a set delay after trigger is true
   *
   * @param seconds
   * @param trigger
   */
  public DelayedBoolean(double seconds, BooleanSupplier trigger) {
    delayTime = seconds;
    this.trigger = trigger;
  }

  public boolean get() {
    if (trigger.getAsBoolean() && !hasTriggered) {
      hasTriggered = true;
      timer.restart();
    }
    return timer.get() > delayTime;
  }
}
