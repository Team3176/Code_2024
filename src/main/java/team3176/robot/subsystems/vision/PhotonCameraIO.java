package team3176.robot.subsystems.vision;

import org.littletonrobotics.junction.AutoLog;
import org.photonvision.PhotonCamera;
import org.photonvision.common.dataflow.structures.Packet;
import org.photonvision.targeting.PhotonPipelineResult;

public class PhotonCameraIO {
  private PhotonCamera cam;
  private Packet packet = new Packet(1);

  @AutoLog
  public static class PhotonCameraInputs {
    byte[] rawBytes = new byte[1];
    boolean isConnected = false;
    double timestamp = 0.0;
  }

  public PhotonCameraIO(String name) {
    this.cam = new PhotonCamera(name);
    // PhotonPipelineResult.serde.pack(packet, new PhotonPipelineResult());
  }

  public void updateInputs(PhotonCameraInputs inputs) {
    if (this.cam.isConnected()) {
      PhotonPipelineResult result = this.cam.getLatestResult();
      packet = new Packet(result.getPacketSize());
      PhotonPipelineResult.serde.pack(packet, result);
      inputs.timestamp = result.getTimestampSeconds();
    } else {
      PhotonPipelineResult result = new PhotonPipelineResult();
      packet = new Packet(result.getPacketSize());
      PhotonPipelineResult.serde.pack(packet, result);
    }
    inputs.rawBytes = packet.getData();
    inputs.isConnected = this.cam.isConnected();
  }

  public PhotonPipelineResult getResult(byte[] b) {
    if (b == null || b.length == 1) {
      return new PhotonPipelineResult();
    }
    return PhotonPipelineResult.serde.unpack(new Packet(b));
  }

  public PhotonCamera getCamera() {
    return this.cam;
  }
}
