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
import team3176.robot.subsystems.RobotStateIO;
import team3176.robot.Constants;
import team3176.robot.Constants.Mode;
import org.littletonrobotics.junction.Logger;

public class Elevator extends SubsystemBase {
  private static Elevator instance;
  private final ElevatorIOFalcon io;
  private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();
  private final DigitalInput limitswitch1 = new DigitalInput(3);
  private final DigitalInput limitswitch2 = new DigitalInput(4);

   
   
  private Elevator(ElevatorIOFalcon io) {
    this.io = io;
    SmartDashboard.putNumber("Arm_kp", SuperStructureConstants.ARM_kP);
    SmartDashboard.putNumber("Arm_Kg", SuperStructureConstants.ARM_kg);
    SmartDashboard.putNumber("arm_angle", 0.0);
    SmartDashboard.putNumber("arm_height", 1.18);
   }

  public void setCoastMode() {
    io.setCoastMode(true);
  }

  public void setBrakeMode() {
    io.setCoastMode(false);
  }

  public void setElevatorMotor(double position){

  //if(limitswitch1.get()  || limitswitch2.get()) {
  if(limitswitch1.get() ) {
    System.out.println("Limitswitch value:"+limitswitch1.get());
      io.set(position);
    } else { 
      io.stop();
    }
  }
    
 
    
  @Override
  public void periodic() {
  }

  public static Elevator getInstance() {
    if (instance == null) {
      if(Constants.getMode() == Mode.REAL) {
        instance = new Elevator(new ElevatorIOFalcon() {});
      }
    }
  return instance;
  }
}
