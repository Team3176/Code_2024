package team3176.robot.subsystems.drivetrain;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Unit;
import team3176.robot.constants.SwervePodHardwareID;
import team3176.robot.util.God.Units3176;

public class SwervePodIOFalconSpark implements SwervePodIO{
    private static final double AZIMUTH_GEAR_RATIO = 70.0 / 1.0; // Is the Versa gearbox btwn motor & encoder
    public static final double THRUST_GEAR_RATIO = (14.0/22.0) * (15.0/45.0);  

    public static final double AZIMUTH_ENCODER_UNITS_PER_REVOLUTION = 4096;
    public static final double THRUST_ENCODER_UNITS_PER_REVOLUTION = 2048;
    
    private CANSparkMax turnSparkMax;
    private TalonFX thrustFalcon;
    private CANCoder azimuthEncoder;

    //public static final double FEET2TICS = 12.0 * (1.0/ (DrivetrainConstants.WHEEL_DIAMETER_INCHES * Math.PI)) * (1.0 /DrivetrainConstants.THRUST_GEAR_RATIO) * DrivetrainConstants.THRUST_ENCODER_UNITS_PER_REVOLUTION;
    public SwervePodIOFalconSpark(SwervePodHardwareID id,int sparkMaxID) {
        turnSparkMax = new CANSparkMax(sparkMaxID, MotorType.kBrushless);
        thrustFalcon = new TalonFX(id.THRUST_CID);
        //reset the motor controllers
        thrustFalcon.configFactoryDefault();
        turnSparkMax.restoreFactoryDefaults();

        thrustFalcon.configClosedloopRamp(0.5);   
        thrustFalcon.setInverted(false);
        thrustFalcon.config_kP(0, 0.03);
        thrustFalcon.config_kI(0, 0.0);
        thrustFalcon.config_kD(0, 0.0);
        thrustFalcon.config_kF(0, 0.045);
        
        //turnSparkMax.setOpenLoopRampRate(0.0);
        turnSparkMax.setSmartCurrentLimit(25);
        turnSparkMax.setInverted(true);

        azimuthEncoder = new CANCoder(id.CANCODER_CID);
        azimuthEncoder.configAbsoluteSensorRange(AbsoluteSensorRange.Signed_PlusMinus180);
        
        azimuthEncoder.configMagnetOffset(id.OFFSET);
        azimuthEncoder.configSensorDirection(true,100);
        
    }
    @Override
    public void updateInputs(SwervePodIOInputs inputs) {
        inputs.drivePositionRad = thrustFalcon.getSelectedSensorPosition() * (THRUST_GEAR_RATIO) * 1.0/THRUST_ENCODER_UNITS_PER_REVOLUTION* 2 * Math.PI;
        inputs.driveVelocityRadPerSec = thrustFalcon.getSelectedSensorVelocity() * (THRUST_GEAR_RATIO) * 1.0/THRUST_ENCODER_UNITS_PER_REVOLUTION * 10 * 2 * Math.PI;
        inputs.driveAppliedVolts = thrustFalcon.getMotorOutputVoltage();
        inputs.driveCurrentAmpsStator = new double[] {thrustFalcon.getStatorCurrent()};
        inputs.driveCurrentAmpsSupply = new double[] {thrustFalcon.getSupplyCurrent()};
        inputs.driveTempCelcius = new double[] {thrustFalcon.getTemperature()};

        inputs.turnAbsolutePositionDegrees = azimuthEncoder.getAbsolutePosition();
        
        inputs.turnVelocityRPM = turnSparkMax.getEncoder().getVelocity();
        inputs.turnAppliedVolts = turnSparkMax.getAppliedOutput() * turnSparkMax.getBusVoltage();
        inputs.turnCurrentAmps = new double[] {turnSparkMax.getOutputCurrent()};
        inputs.turnTempCelcius = new double[] {turnSparkMax.getMotorTemperature()};
    }

    @Override
    public void setDrive(double velMetersPerSecond) {
        double velTicsPer100ms = velMetersPerSecond * (1.0/ (SwervePod.WHEEL_DIAMETER * Math.PI)) * (1.0 /THRUST_GEAR_RATIO) * THRUST_ENCODER_UNITS_PER_REVOLUTION  * .1;;
        thrustFalcon.set(TalonFXControlMode.Velocity, velTicsPer100ms);
    }

    /** Run the turn motor at the specified voltage. */
    @Override
    public void setTurn(double percent) {
        turnSparkMax.set(percent);
    }

    /** Enable or disable brake mode on the drive motor. */
    @Override
    public void setDriveBrakeMode(boolean enable) {
        if(enable){
            thrustFalcon.setNeutralMode(NeutralMode.Brake);
        } else {
            thrustFalcon.setNeutralMode(NeutralMode.Coast);
        }
    }

    /** Enable or disable brake mode on the turn motor. */
    @Override
    public void setTurnBrakeMode(boolean enable) {
        if(enable) {
            turnSparkMax.setIdleMode(IdleMode.kBrake);
        } else {
            turnSparkMax.setIdleMode(IdleMode.kCoast);
        }
    }
}
