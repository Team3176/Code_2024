package team3176.robot.subsystems.vision;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Pose3d;

public interface VisionCubeChaseIO {
    
    @AutoLog
    public static class VisionCubeChaseInputs {
        //public Pose3d rfovBlue = new Pose3d();
        //public Pose3d rfovRed = new Pose3d();
        public double rLatency = 0.0;
        public double rNumTags = 0;
        public boolean rValid = false;
        public double rTx;
        //constructor if needed for some inputs
        VisionCubeChaseInputs() {
        }
    }
    /** Updates the set of loggable inputs. */
  public default void updateInputs(VisionCubeChaseInputs inputs) {}
}
