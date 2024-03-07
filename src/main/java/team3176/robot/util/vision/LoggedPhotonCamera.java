package team3176.robot.util.vision;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.PubSubOption;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;

public class LoggedPhotonCamera extends PhotonCamera {
  LoggedPacketSubscriber<PhotonPipelineResult> resultSubscriber;

  public LoggedPhotonCamera(String cameraName) {
    super(cameraName);
    NetworkTable cameraTable = this.getCameraTable();
    var rawBytesEntry =
        cameraTable
            .getRawTopic("rawBytes")
            .subscribe(
                "rawBytes", new byte[] {}, PubSubOption.periodic(0.01), PubSubOption.sendAll(true));
    resultSubscriber =
        new LoggedPacketSubscriber<>(
            rawBytesEntry, PhotonPipelineResult.serde, new PhotonPipelineResult());
  }

  @Override
  public PhotonPipelineResult getLatestResult() {

    var ret = resultSubscriber.get();

    // Set the timestamp of the result.
    // getLatestChange returns in microseconds, so we divide by 1e6 to convert to seconds.
    ret.setTimestampSeconds(
        (resultSubscriber.subscriber.getLastChange() / 1e6) - ret.getLatencyMillis() / 1e3);

    // Return result.
    return ret;
  }
}
