package team3176.robot.subsystems.vision;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import team3176.robot.subsystems.vision.VisionCubeChaseIO.VisionCubeChaseInputs;

public class VisionCubeChase extends SubsystemBase{
    private VisionCubeChaseIO io;
    private Pose3d bestVisionPose3d;
    private Pose2d bestVisionPose2d;
    private static VisionCubeChase instance;
    private final VisionCubeChaseInputs inputs = new VisionCubeChaseInputs();
    private VisionCubeChase(VisionCubeChaseIO io) {
        this.io = io;
    }
    public static VisionCubeChase getInstance() {
        if (instance == null) {
            instance = new VisionCubeChase(new VisionCubeChaseIOLime());
        }
        return instance;
    }
    public Pose3d getPose3d() {
        return bestVisionPose3d;
    }
    public Pose2d getPose2d() {
        return bestVisionPose2d;
    }

    public boolean isValid() {
        return inputs.rValid;
    }

    public double getTx() {
        return inputs.rTx;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        //Logger.getInstance().processInputs("VisionCubeChase", inputs);
        
        //Logger.recordOutput("Vision/bestPose",bestVisionPose3d);
        
        
    }
}
