package team3176.robot.subsystems.vision;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import team3176.robot.Constants;
import team3176.robot.Constants.Mode;
import team3176.robot.subsystems.vision.VisionDualIO.VisionDualInputs;
import team3176.robot.subsystems.vision.VisionDualIO;
public class VisionDual extends SubsystemBase{
    private VisionDualIO io;
    private VisionDualInputs inputs;
    private Pose3d bestVisionPose3d;
    private Pose2d bestVisionPose2d;
    private enum blendModes {RIGHT,LEFT,BOTH};
    private blendModes mode;
    private static VisionDual instance;
    private VisionDual(VisionDualIO io) {
        this.io = io;
        this.inputs = new VisionDualInputs(); 
        mode = blendModes.LEFT;
    }
    public static VisionDual getInstance() {
        if (instance == null) {
            if(Constants.getMode() == Mode.REAL) {
                instance = new VisionDual(new VisionDualIOLime());
            } else {
                instance = new VisionDual(new VisionDualIO() {});
            }
            
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
        return inputs.lValid || inputs.rValid;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.getInstance().processInputs("Vision", inputs);
        boolean isRed = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red;
        Pose3d rPose = isRed ? inputs.rfovRed : inputs.rfovBlue;
        Pose3d lPose = isRed ? inputs.lfovRed : inputs.lfovBlue;
        if(inputs.lValid && inputs.rValid) {
            //possible both good
            if(inputs.lNumTags > inputs.rNumTags) {
                mode = blendModes.LEFT;
            } 
            else if (inputs.rNumTags > inputs.lNumTags) {
                mode = blendModes.RIGHT;
            }
            else {
                mode = blendModes.BOTH;
            }
        }
        else if (inputs.lValid) {
            mode = blendModes.LEFT;
        }
        else {
            mode = blendModes.RIGHT;
        }
        
        switch(mode){
            case BOTH:
                bestVisionPose3d  = lPose.interpolate(rPose, 0.5);       
                break;
            case LEFT:
                bestVisionPose3d = lPose;
                break;
            case RIGHT:
                bestVisionPose3d = rPose;
                break;
        }
        bestVisionPose2d = bestVisionPose3d.toPose2d();
        Logger.recordOutput("Vision/bestPose",bestVisionPose3d);
        
        
        // wildstang 111 code for refence
        // if (rtv > 0.0 && ltv > 0.0){
        //     if (lnumtargets > rnumtargets){
        //     return (getLeftVertical()+offset*LC.OFFSET_VERTICAL) * LC.VERT_AUTOAIM_P;
        //     } else if (rnumtargets > lnumtargets){
        //         return (getRightVertical()+offset*LC.OFFSET_VERTICAL) * LC.VERT_AUTOAIM_P;
        //     }
        //     return LC.VERT_AUTOAIM_P * (offset*LC.OFFSET_VERTICAL + (getLeftVertical() + getRightVertical())/2.0);
        // } else if (ltv > 0.0){
        //     return (getLeftVertical()+offset*LC.OFFSET_VERTICAL) * LC.VERT_AUTOAIM_P;
        // } else {
        //     return (getRightVertical()+offset*LC.OFFSET_VERTICAL) * LC.VERT_AUTOAIM_P;
        // }
    }
}
