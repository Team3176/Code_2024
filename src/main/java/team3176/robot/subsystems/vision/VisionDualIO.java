package team3176.robot.subsystems.vision;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;

public interface VisionDualIO {
    
    public static class VisionDualInputs implements LoggableInputs {
        public Pose3d rfovBlue = new Pose3d();
        public Pose3d rfovRed = new Pose3d();
        public Pose3d lfovBlue = new Pose3d();
        public Pose3d lfovRed = new Pose3d();
        public double rLatency = 0.0;
        public double lLatency = 0.0;
        public double rNumTags = 0;
        public double lNumTags = 0;
        public boolean rValid = false;
        public boolean lValid = false;
        private double[] blankPoseArray = {0.0,0.0,0.0,0.0,0.0,0.0};
        //constructor if needed for some inputs
        VisionDualInputs() {
        }
        private double[] toLogPose3(Pose3d p) {
          double[] poseArray = {p.getX(),p.getY(),p.getZ(),p.getRotation().getX(),p.getRotation().getY(),p.getRotation().getZ()};
          return poseArray;
        }
        private Pose3d fromLogPose3(double [] a) {
          Rotation3d r = new Rotation3d(a[3], a[4], a[5]);
          Pose3d pose = new Pose3d(a[0], a[1], a[2], r);
          return pose;
        }
        @Override
        public void toLog(LogTable table) {
          table.put("rfovBlue", toLogPose3(rfovBlue));
          table.put("lfovBlue", toLogPose3(lfovBlue));
          table.put("rfovRed", toLogPose3(rfovRed));
          table.put("rfovRed", toLogPose3(lfovRed));
          table.put("RLatency", rLatency);
          table.put("LLatency", lLatency);
          table.put("rNumTags",rNumTags);
          table.put("lNumTags",lNumTags);
          table.put("RValid", rValid);
          table.put("LValid", lValid);
          
        }
        @Override
        public void fromLog(LogTable table) {
          rfovBlue=fromLogPose3(table.getDoubleArray("rfovBlue",blankPoseArray));
          lfovBlue=fromLogPose3(table.getDoubleArray("lfovBlue",blankPoseArray));
          rfovRed=fromLogPose3(table.getDoubleArray("rfovRed",blankPoseArray));
          lfovRed=fromLogPose3(table.getDoubleArray("lfovRed",blankPoseArray));
          rLatency = table.getDouble("RLatency", rLatency);
          lLatency = table.getDouble("LLatency", lLatency);
          rValid = table.getBoolean("RValid", rValid);
          lValid = table.getBoolean("LValid", lValid);
          rNumTags = table.getDouble("rNumTags", rNumTags);
          lNumTags = table.getDouble("lNumTags", lNumTags);
        }
    }
    /** Updates the set of loggable inputs. */
  public default void updateInputs(VisionDualInputs inputs) {}
}
