package team3176.robot.util.vision;

import edu.wpi.first.networktables.RawSubscriber;
import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;
import org.photonvision.common.dataflow.structures.Packet;
import org.photonvision.common.dataflow.structures.PacketSerde;

public class LoggedPacketSubscriber<T> implements AutoCloseable {
  @AutoLog
  public static class PacketInputs {
    byte[] rawBytes = new byte[1];
  }

  public final RawSubscriber subscriber;
  private final PacketSerde<T> serde;
  private final T defaultValue;
  private final Packet packet = new Packet(1);
  private PacketInputsAutoLogged inputs;

  public LoggedPacketSubscriber(RawSubscriber subscriber, PacketSerde<T> serde, T defaultValue) {
    this.subscriber = subscriber;
    this.serde = serde;
    this.defaultValue = defaultValue;
    inputs = new PacketInputsAutoLogged();
  }

  public T get() {
    packet.clear();
    if (!Logger.hasReplaySource()) {
      inputs.rawBytes = subscriber.get(new byte[] {});
    }
    Logger.processInputs(subscriber.getTopic().getName(), inputs);
    if (inputs.rawBytes.length == 1) {
      return defaultValue;
    }
    packet.setData(inputs.rawBytes);
    if (packet.getSize() < 1) return defaultValue;

    return serde.unpack(packet);
  }

  @Override
  public void close() {
    subscriber.close();
  }
}
