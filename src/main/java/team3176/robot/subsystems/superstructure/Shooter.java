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

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.NeutralOut;
import org.littletonrobotics.junction.Logger;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.configs.TalonFXConfiguration;

public class Shooter extends SubsystemBase {
    private static Shooter instance;
    private final ShooterIO io;
    private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();
    private final VelocityVoltage m_voltageVelocity = new VelocityVoltage(0, 0, true, 0, 0, false, false, false);  
    private final NeutralOut m_brake = new NeutralOut();  
    private final PIDController pivotPIDController;
    private final TalonFX m_fx = new TalonFX(0);
    public enum States {OPEN_LOOP,CLOSED_LOOP}
    private States currentState = States.OPEN_LOOP;
    private double armSetpointAngleRaw = SuperStructureConstants.SHOOTER_ZERO_POS;
    private Mechanism2d mech = new Mechanism2d(3,3);
    private MechanismRoot2d root = mech.getRoot("armRoot", 0.0, 0.42);
    private MechanismLigament2d armSholder = root.append(new MechanismLigament2d("armLigament",.4,90));
    //private MechanismLigament2d simSholder = root.append(new MechanismLigament2d("simLigament",3,90,10,new Color8Bit(Color.kAqua)));
    private MechanismLigament2d armElbow = armSholder.append(new MechanismLigament2d("armELigament",0.5,90));
    private TalonFXConfiguration configsWheelPort;
    private TalonFXConfiguration configsWheelStarbrd;
    private TalonFXConfiguration configsPivot;
//    private final TunablePID pivotPIDController;
    private Rotation2d pivotSetpoint = new Rotation2d();

    private Shooter(ShooterIO io) {
        this.io = io;
        this.pivotPIDController = new PIDController(SuperStructureConstants.ANGLER_kP, SuperStructureConstants.ANGLER_kI, SuperStructureConstants.ANGLER_kD);
        SmartDashboard.putNumber("Arm_kp", SuperStructureConstants.SHOOTER_kP);
        SmartDashboard.putNumber("Arm_Kg", SuperStructureConstants.SHOOTER_kg);
        SmartDashboard.putNumber("arm_angle", 0.0);
        SmartDashboard.putNumber("arm_height", 1.18);
      TalonFXConfiguration configs = new TalonFXConfiguration();
  
      /* Voltage-based velocity requires a feed forward to account for the back-emf of the motor */
      configs.Slot0.kP = 0.11; // An error of 1 rotation per second results in 2V output
      configs.Slot0.kI = 0.5; // An error of 1 rotation per second increases output by 0.5V every second
      configs.Slot0.kD = 0.0001; // A change of 1 rotation per second squared results in 0.01 volts output
      configs.Slot0.kV = 0.12; // Falcon 500 is a 500kV motor, 500rpm per V = 8.333 rps per V, 1/8.33 = 0.12 volts / Rotation per second
      // Peak output of 8 volts
      configs.Voltage.PeakForwardVoltage = 8;
      configs.Voltage.PeakReverseVoltage = -8;
    }

  public static Shooter getInstance() {
    if (instance == null) {
      instance = new Shooter(new ShooterIO() {});
    }
    return instance;
  }


    
    private void setPIDPosition(double position) {
        double pivotVoltage = pivotPIDController.calculate(inputs.pivotPosition.getRadians(), pivotSetpoint.getRadians());
        pivotVoltage = MathUtil.clamp(pivotVoltage,-12,12);
        io.setPivotVoltage(pivotVoltage);
    }

    public Command pivotSetPositionOnce(double angleInDegrees) {
        return this.runOnce(() -> {
            this.pivotSetpoint = Rotation2d.fromDegrees(angleInDegrees);}).withName("armSetPosition"+angleInDegrees);
    }

    

    public void robotInit() {
    }
    
    
    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Shooter", inputs);
        Logger.recordOutput("Shooter/desired", pivotSetpoint);
        //simSholder.setAngle(Rotation2d.fromDegrees(inputs.Position -90 - SuperStructureConstants.ARM_SIM_OFFSET))
       
        //SmartDashboard.putNumber("Arm_Position", armEncoder.getAbsolutePosition());
        //SmartDashboard.putNumber("Arm_Position_Relative", armEncoder.getAbsolutePosition() - SuperStructureConstants.ARM_ZERO_POS);
        if(this.currentState == States.CLOSED_LOOP) {
            this.armSetpointAngleRaw = MathUtil.clamp(this.armSetpointAngleRaw, SuperStructureConstants.ANGLER_ZERO_POS, SuperStructureConstants.ANGLER_HIGH_POS);
            Logger.recordOutput("Arm/position_error", this.pivotPIDController.getPositionError());
            setPIDPosition(armSetpointAngleRaw);
        }
        Logger.recordOutput("Shooter/position_error", this.pivotPIDController.getPositionError());
    }
    
    public void ShooterPIDVelocity(double velocity) {

        double desiredRotationsPerSecond = velocity * 50; // Go for plus/minus 10 rotations per second
        
        /* Use voltage velocity */
        m_fx.setControl(m_voltageVelocity.withVelocity(desiredRotationsPerSecond));
        /* Disable the motor instead */
        //m_fx.setControl(m_brake);
    }
        
}

