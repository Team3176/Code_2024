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
import team3176.robot.Constants;
import team3176.robot.Constants.Mode;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {
    private static Intake instance;
    private final IntakeIO io;
    private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();
    
    private Intake(IntakeIO io) {
        this.io = io;
    }

    public void setCoastMode() {
        io.setCoastMode(true);
    }

    public void setBrakeMode() {
        io.setCoastMode(false);
    } 

    public Command runIntake(double percentoutput){
        return this.runEnd(() -> set(percentoutput), () -> set(0.0));
    }

    public void set(double outputpercent){
        io.set(outputpercent);
    }

    public static Intake getInstance() {
        if (instance == null){
            if(Constants.getMode() != Mode.SIM) {
                instance = new Intake(new IntakeIOSpark() {});
            } else {
                instance = new Intake(new IntakeIOSim() {});
            }
            
        }
        return instance;
    }
    
    
    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Intake", inputs);
        //Logger.recordOutput("Arm/mech2d", mech);
    }
}
