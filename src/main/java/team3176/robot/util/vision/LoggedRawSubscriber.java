package team3176.robot.util.vision;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.PubSubOption;
import edu.wpi.first.networktables.RawSubscriber;
import edu.wpi.first.networktables.RawTopic;
import edu.wpi.first.networktables.TimestampedRaw;
import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;

/*
 * very experimental to be used with photonvision
 */
public class LoggedRawSubscriber implements RawSubscriber {
  @AutoLog
  public static class rawInputs {
    byte[] raw = new byte[1];
  }

  RawSubscriber rawSub;
  rawInputsAutoLogged inputs;
  String LogName;

  public LoggedRawSubscriber(NetworkTable cameraTable) {
    LogName = cameraTable.getPath();
    rawSub =
        cameraTable
            .getRawTopic("rawBytes")
            .subscribe(
                "rawBytes", new byte[] {}, PubSubOption.periodic(0.01), PubSubOption.sendAll(true));
    inputs = new rawInputsAutoLogged();
  }

  @Override
  public boolean exists() {
    return rawSub.exists();
  }

  @Override
  public long getLastChange() {
    return rawSub.getLastChange();
  }

  @Override
  public void close() {
    rawSub.close();
  }

  @Override
  public boolean isValid() {
    return rawSub.isValid();
  }

  @Override
  public int getHandle() {
    return rawSub.getHandle();
  }

  @Override
  public RawTopic getTopic() {
    return rawSub.getTopic();
  }

  @Override
  public byte[] get() {
    if (!Logger.hasReplaySource()) {
      inputs.raw = rawSub.get();
    }
    Logger.processInputs(LogName + "/rawLog", inputs);
    return inputs.raw;
  }

  @Override
  public byte[] get(byte[] defaultValue) {
    if (!Logger.hasReplaySource()) {
      inputs.raw = rawSub.get(defaultValue);
    }
    Logger.processInputs(LogName + "/rawLog", inputs);
    return inputs.raw;
  }

  @Override
  public TimestampedRaw getAtomic() {
    return rawSub.getAtomic();
  }

  @Override
  public TimestampedRaw getAtomic(byte[] defaultValue) {
    return rawSub.getAtomic(defaultValue);
  }

  @Override
  public TimestampedRaw[] readQueue() {
    return rawSub.readQueue();
  }

  @Override
  public byte[][] readQueueValues() {
    return rawSub.readQueueValues();
  }
}
