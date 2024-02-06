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
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import team3176.robot.constants.SuperStructureConstants;
import team3176.robot.util.TunablePID;
import team3176.robot.Constants;
import team3176.robot.Constants.Mode;
import com.ctre.phoenix6.controls.NeutralOut;
import org.littletonrobotics.junction.Logger;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.configs.TalonFXConfiguration;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.NeutralOut;
import org.littletonrobotics.junction.Logger;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

public class Shooter extends SubsystemBase {
    private static Shooter instance;
    private final ShooterIO io;
    private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();
    private TalonFXConfiguration configsWheelPort = new TalonFXConfiguration();
    private TalonFXConfiguration configsWheelStarbrd = new TalonFXConfiguration();
        
    private final TunablePID pivotPIDController;
    private Rotation2d pivotSetpoint = new Rotation2d();



    private Shooter(ShooterIO io) {
        this.io = io;
        this.pivotPIDController = new TunablePID("shooter/pid",0.0,0.0,0.0);
        pivotPIDController.setTolerance(Units.degreesToRadians(3.0));
          
        /* Voltage-based velocity requires a feed forward to account for the back-emf of the motor */
        configsWheelPort.Slot0.kP = 0.11; // An error of 1 rotation per second results in 2V output
        configsWheelPort.Slot0.kI = 0.5; // An error of 1 rotation per second increases output by 0.5V every second
        configsWheelPort.Slot0.kD = 0.0001; // A change of 1 rotation per second squared results in 0.01 volts output
        configsWheelPort.Slot0.kV = 0.12; // Falcon 500 is a 500kV motor, 500rpm per V = 8.333 rps per V, 1/8.33 = 0.12 volts / Rotation per second
        // Peak output of 8 volts
        configsWheelPort.Voltage.PeakForwardVoltage = 8;
        configsWheelPort.Voltage.PeakReverseVoltage = -8;

        configsWheelStarbrd.Slot0.kP = 0.11; // An error of 1 rotation per second results in 2V output
        configsWheelStarbrd.Slot0.kI = 0.5; // An error of 1 rotation per second increases output by 0.5V every second
        configsWheelStarbrd.Slot0.kD = 0.0001; // A change of 1 rotation per second squared results in 0.01 volts output
        configsWheelStarbrd.Slot0.kV = 0.12; // Falcon 500 is a 500kV motor, 500rpm per V = 8.333 rps per V, 1/8.33 = 0.12 volts / Rotation per second
        // Peak output of 8 volts
        configsWheelStarbrd.Voltage.PeakForwardVoltage = 8;
        configsWheelStarbrd.Voltage.PeakReverseVoltage = -8;


    }


    public static Shooter getInstance() {
        if (instance == null){
            if (Constants.getMode() != Mode.SIM) {
                instance = new Shooter(new ShooterIOFalcon() {});
            } else {
                return instance;
            }
        }
    }
    
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
        m_mechanisms.update(m_fx.getPosition(), m_fx.getVelocity());
    }
    
    public void ShooterPIDVelocity(double velocity) {

        double desiredRotationsPerSecond = velocity * 50; // Go for plus/minus 10 rotations per second
        
        /* Use voltage velocity */
        m_fx.setControl(m_voltageVelocity.withVelocity(desiredRotationsPerSecond));
        return m_fx.getaver
        }
        else {
        /* Disable the motor instead */
        m_fx.setControl(m_brake);
        
  }
}

        public void ControllerTovVelocity(double JoystickButtonbuttonimput)
        {
            if( JoystickButtonbuttonimput < 0.1 && JoystickButtonbuttonimput > -0.1)
                JoystickButtonbuttonimput = 0;
            double velocity;
            velocity = JoystickButtonbuttonimput * 50;
            
        }