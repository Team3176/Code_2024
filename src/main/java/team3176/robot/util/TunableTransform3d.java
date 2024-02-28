package team3176.robot.util;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;

public class TunableTransform3d {
  private String name;
  private LoggedTunableNumber xLog;
  private LoggedTunableNumber yLog;
  private LoggedTunableNumber zLog;
  private LoggedTunableNumber rollLog;
  private LoggedTunableNumber pitchLog;
  private LoggedTunableNumber yawLog;
  public Transform3d transform;

  public TunableTransform3d(
      String name, double x, double y, double z, double roll, double pitch, double yaw) {
    transform = new Transform3d(x, y, z, new Rotation3d(roll, pitch, yaw));
    this.name = name;
    this.xLog = new LoggedTunableNumber(name + "/x", x);
    this.yLog = new LoggedTunableNumber(name + "/y", y);
    this.zLog = new LoggedTunableNumber(name + "/z", z);
    this.rollLog = new LoggedTunableNumber(name + "/roll", roll);
    this.pitchLog = new LoggedTunableNumber(name + "/pitch", pitch);
    this.yawLog = new LoggedTunableNumber(name + "/yaw", yaw);
  }

  public void checkParemeterUpdate() {
    LoggedTunableNumber.ifChanged(
        hashCode(),
        () -> {
          this.transform =
              new Transform3d(xLog.get(), yLog.get(), zLog.get(), transform.getRotation());
        },
        xLog,
        yLog,
        zLog);
    LoggedTunableNumber.ifChanged(
        hashCode(),
        () -> {
          this.transform =
              new Transform3d(
                  transform.getTranslation(),
                  new Rotation3d(rollLog.get(), pitchLog.get(), yawLog.get()));
        },
        rollLog,
        pitchLog,
        yawLog);
  }
}
