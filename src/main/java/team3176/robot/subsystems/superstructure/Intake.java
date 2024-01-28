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
import edu.wpi.first.units.Velocity;
import edu.wpi.first.wpilibj.DigitalInput;
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
    private final DigitalInput linebreak1 = new DigitalInput(5);
    private final DigitalInput linebreak2 = new DigitalInput(6);

    private Intake(IntakeIO io) {
        this.io = io;
        SmartDashboard.putNumber("Intake_kp", SuperStructureConstants.INTAKE_kP);
        SmartDashboard.putNumber("Intake_Kg", SuperStructureConstants.INTAKE_kg);
        SmartDashboard.putNumber("Intake_angle", 0.0);
        SmartDashboard.putNumber("Intake_velocity", 0.0);
    }

    public void setCoastMode() {
        io.setCoastMode(true);
    }

    public void setBrakeMode() {
        io.setCoastMode(false);
    } 
    
    public void setIntakeMotor(double velocity){

        if(linebreak1.get()  || linebreak2.get()) {
            System.out.println("Limitswitch value:"+linebreak1.get());
              io.setRoller(velocity);
            } else { 
              io.stopRoller();
            }
    }

    public Command moveIntake(double velocity){
        return this.run(() -> setIntakeMotor(velocity));
    }
    
    
    
    @Override
    public void periodic() {}

    public static Intake getInstance() {
        if (instance == null) {
          instance = new Intake(new IntakeIO() {});
        }
        return instance;
      }
    
}
