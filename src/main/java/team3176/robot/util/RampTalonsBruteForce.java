package team3176.robot.util;

import com.ctre.phoenix6.hardware.TalonFX;

/*
This is so brute force.  It's just philosophically wrong.
*/

/**
 * This class is a utility class for TalonFX motor controllers. It contains methods to brute force
 * apply ramping rates when called. These ramps are applied over 1 sec (ie stepped over 1sec)
 */
public class RampTalonsBruteForce {

  int loopsPer1Sec = 50;

  public void rampOver1SecStepwise(TalonFX motor, double currentVoltage, double targetVoltage) {
    double rampStepSize = (targetVoltage - currentVoltage) / 20;
    for (int i = 0; i < loopsPer1Sec; i++) {
      motor.setVoltage(currentVoltage + (rampStepSize * i));
      try {
        Thread.sleep(20);
      } catch (InterruptedException e) {
        e.printStackTrace();
      }
    }
  }

  /**
   * This class is a utility class for TalonFX motor controllers. It contains methods to brute force
   * apply ramping rates when called. These ramps are applied over the SUPPLIED time (ie stepped
   * over the passed value that "time" in seconds)
   */
  public void rampOverVarSecStepwise(
      TalonFX motor, double currentVoltage, double targetVoltage, double time) {
    double rampStepSize = (targetVoltage - currentVoltage) / 20;
    for (int i = 0; i < loopsPer1Sec; i++) {
      motor.setVoltage(currentVoltage + (rampStepSize * i));
      try {
        Thread.sleep(20);
      } catch (InterruptedException e) {
        e.printStackTrace();
      }
    }
  }
}
