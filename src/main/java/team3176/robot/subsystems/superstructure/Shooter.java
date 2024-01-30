package team3176.robot.subsystems.superstructure;

import java.util.function.DoubleSupplier;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Unit;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import team3176.robot.constants.SuperStructureConstants;
import team3176.robot.util.TunablePID;
import team3176.robot.Constants;
import team3176.robot.Constants.Mode;
import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {
    private static Shooter instance;
    private final ShooterIO io;
    private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();
        
    private final TunablePID pivotPIDController;
    private Rotation2d pivotSetpoint = new Rotation2d();
    private Shooter(ShooterIO io) {
        this.io = io;
        this.pivotPIDController = new TunablePID("shooter/pid",0.0,0.0,0.0);
        pivotPIDController.setTolerance(Units.degreesToRadians(3.0));
    }

    public static Shooter getInstance() {
        if (instance == null){
            if(Constants.getMode() != Mode.SIM) {
                instance = new Shooter(new ShooterIOFalcon() {});
            } else {
                instance = new Shooter(new ShooterIOSim() {});
            }
            
        }
        return instance;
    }
    
    /**
     * 
     * @param desiredAngle in degrees in Encoder Frame
     */
    private void PIDPositionPeriodic() {
        double pivotVoltage = pivotPIDController.calculate(inputs.pivotPosition.getRadians(), pivotSetpoint.getRadians());
        pivotVoltage = MathUtil.clamp(pivotVoltage,-12,12);
        io.setPivotVoltage(pivotVoltage);
    }
    public Command pivotSetPositionOnce(double angleInDegrees) {
        return this.runOnce(() -> {
            this.pivotSetpoint = Rotation2d.fromDegrees(angleInDegrees);}).withName("armSetPosition"+angleInDegrees);
    }

    
    
    
    @Override
    public void periodic() {
        io.updateInputs(inputs);
        pivotPIDController.checkParemeterUpdate();
        Logger.processInputs("Shooter", inputs);
        Logger.recordOutput("Shooter/desired", pivotSetpoint);
        //simSholder.setAngle(Rotation2d.fromDegrees(inputs.Position -90 - SuperStructureConstants.ARM_SIM_OFFSET))
       
        //SmartDashboard.putNumber("Arm_Position", armEncoder.getAbsolutePosition());
        //SmartDashboard.putNumber("Arm_Position_Relative", armEncoder.getAbsolutePosition() - SuperStructureConstants.ARM_ZERO_POS);
       
        
        Logger.recordOutput("Shooter/position_error", this.pivotPIDController.getPositionError());
        PIDPositionPeriodic();
    }
}
