package team3176.robot.util;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;

public class SimulationUtils {
    /**
     * sets volts to zero if not enabled otherwise clamps to between -12 and 12v
     */
    public static double simulationVoltsClamp(double volts) {
    if (!DriverStation.isEnabled()) {
      volts = 0.0;
    }
    return MathUtil.clamp(volts, -12, 12);
  }
}
