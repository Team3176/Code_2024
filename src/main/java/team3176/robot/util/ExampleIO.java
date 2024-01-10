package team3176.robot.util;

import org.littletonrobotics.junction.AutoLog;

public interface ExampleIO {

  /** Contains all of the input data received from hardware. */
  @AutoLog
  public static class ExampleIOInputs {
    public double PositionRad = 0.0;
    public double VelocityRadPerSec = 0.0;
    public double AppliedVolts = 0.0;
    public double[] CurrentAmps = new double[] {};
    public double[] TempCelcius = new double[] {};

    // constructor if needed for some inputs
    ExampleIOInputs() {}
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(ExampleIOInputs inputs) {}

  public default void reset() {}
}
