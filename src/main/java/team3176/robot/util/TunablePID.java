package team3176.robot.util;

import edu.wpi.first.math.controller.PIDController;

public class TunablePID extends PIDController {
  private LoggedTunableNumber kP;
  private LoggedTunableNumber kI;
  private LoggedTunableNumber kD;

  public TunablePID(String LogName, double P, double I, double D) {
    super(P, I, D);
    kP = new LoggedTunableNumber(LogName + "_P", P);
    kI = new LoggedTunableNumber(LogName + "_I", I);
    kD = new LoggedTunableNumber(LogName + "_D", D);
  }

  public void checkParemeterUpdate() {
    LoggedTunableNumber.ifChanged(
        hashCode(),
        () -> {
          setP(kP.get());
          setI(kI.get());
          setD(kD.get());
        },
        kP,
        kD,
        kI);
  }
}
