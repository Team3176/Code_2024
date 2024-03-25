package team3176.robot.util;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;

public class Ramps{

  public void rampOver1SecStepwise(TalonFX motor, double currentVoltage, double targetVoltage) {
    int loopsPer1Sec = 50;
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
