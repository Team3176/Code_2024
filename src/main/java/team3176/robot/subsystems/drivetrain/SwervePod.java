package team3176.robot.subsystems.drivetrain;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Subsystem;
import team3176.robot.Constants;
import team3176.robot.Constants.Mode;
import team3176.robot.util.LoggedTunableNumber;


public class SwervePod implements Subsystem{

    /** Class Object holding the Motor controller for Drive Motor on the SwervePod */
    /** Current value in radians of the azimuthEncoder's position */
    double desiredOptimizedAzimuthPosition;
    private SwerveModuleState desiredState = new SwerveModuleState();
    boolean lastHasResetOccurred;
    public static final double WHEEL_DIAMETER = Units.inchesToMeters(3.00); // Inches


    
    /** Numerical identifier to differentiate between pods.
     *     For 4 Pods:  0 = FrontRight (FR),
     *                  1 = FrontLeft (FL),
     *                  2 = BackLeft (BL),
     *                  3 = BackRight (BR)
     */
    private int id;
    private String idString;
   
    /** Represents the value of the azimuthEncoder reading in radians when positioned with the positive Thrust vector of the Pod's Drive wheel pointing towards front of robot */
    //private double kAzimuthEncoderUnitsPerRevolution;

    //private double kP_Azimuth;
    private LoggedTunableNumber kPAzimuth = new LoggedTunableNumber("kP_azimuth",.012);
    private LoggedTunableNumber kIAzimuth = new LoggedTunableNumber("kI_azimuth",0.0);
    private LoggedTunableNumber kDAzimuth = new LoggedTunableNumber("kD_azimuth",0.0003);
    private LoggedTunableNumber turnMaxpercent = new LoggedTunableNumber("turn_max",0.75);
    private double turnMaxpercentLocal = 0.4;
    private double lastDistance =0.0;
    private double lastDistanceSimNoNoise =0.0;
    private double delta = 0.0;
    private double deltaSimNoNoise = 0.0;
    private LoggedTunableNumber velMax = new LoggedTunableNumber("az_vel");
    private LoggedTunableNumber velAcc = new LoggedTunableNumber("az_acc");

    private final PIDController  turningPIDController;
    //private final ProfiledPIDController m_turningProfiledPIDController;
    //private ProfiledPIDController m_turningPIDController;

    private SwervePodIO io;
    private SwervePodIOInputsAutoLogged inputs = new SwervePodIOInputsAutoLogged();

    public SwervePod(int id, SwervePodIO io) {
        this.id = id;
        this.io = io;
        this.desiredOptimizedAzimuthPosition = 0.0;

        //this.kP_Azimuth = 0.006;

        velMax.initDefault(900);
        velAcc.initDefault(900);

        turningPIDController = new PIDController(kPAzimuth.get(), kIAzimuth.get(), kDAzimuth.get());//,new Constraints(velMax.get(), velAcc.get()));
        turningPIDController.setTolerance(3);
        turningPIDController.enableContinuousInput(-180, 180);
        turningPIDController.setIntegratorRange(-0.1,0.1);
        turningPIDController.setP(this.kPAzimuth.get());
        turningPIDController.setI(this.kIAzimuth.get());
        turningPIDController.setD(this.kDAzimuth.get());
        CommandScheduler.getInstance().registerSubsystem(this);
        
    }


    public void setModule(double speedMetersPerSecond, Rotation2d angle) {
        setModule(new SwerveModuleState(speedMetersPerSecond,angle));
    }

    /**
     *  alternative method for setting swervepod in line with WPILIB standard library
     * @param desiredState 
     */
    public SwerveModuleState setModule(SwerveModuleState desiredState) {
        SwerveModuleState desiredOptimized = SwerveModuleState.optimize(desiredState, Rotation2d.fromDegrees(inputs.turnAbsolutePositionDegrees));
        this.desiredOptimizedAzimuthPosition = desiredOptimized.angle.getDegrees();
        this.desiredState = desiredOptimized;
        return desiredOptimized;
    }   
    /*
     * odometry calls
     */
    public SwerveModulePosition getPosition() {
        double m = (WHEEL_DIAMETER * Math.PI)  *  inputs.drivePositionRad / (2*Math.PI);
        return new SwerveModulePosition(m,Rotation2d.fromDegrees(inputs.turnAbsolutePositionDegrees));
    }
    /** Returns the module state (turn angle and drive velocity). */
    public SwerveModuleState getState() {
        return new SwerveModuleState(getVelocity(), Rotation2d.fromDegrees(inputs.turnAbsolutePositionDegrees));
    }
    public SwerveModulePosition getPositionSimNoNoise() {
        double m = (WHEEL_DIAMETER * Math.PI)  *  inputs.drivePositionSimNoNoise / (2*Math.PI);
        return new SwerveModulePosition(m,Rotation2d.fromDegrees(inputs.turnAbsolutePositionDegreesSimNoNoise));
    }
    public SwerveModulePosition getDelta() {
        return new SwerveModulePosition(this.delta,Rotation2d.fromDegrees(inputs.turnAbsolutePositionDegrees));
    }
    public SwerveModulePosition getDeltaSimNoNoise() {
        return new SwerveModulePosition(this.deltaSimNoNoise,Rotation2d.fromDegrees(inputs.turnAbsolutePositionDegreesSimNoNoise));
    }
    
    public double getVelocity() {
        double wheelVelocity = inputs.driveVelocityRadPerSec / (Math.PI *2) * WHEEL_DIAMETER * Math.PI;   
        return wheelVelocity;
    
    }

    /**
     * Returns current Azimuth of pod in degrees, where 0 is straight forward.
     * @return
     */
    public double getAzimuth() {
        return inputs.turnAbsolutePositionDegrees; 
    }

    public void setThrustCoast() {
        io.setDriveBrakeMode(false);
    }

    public void setThrustBrake() {
        io.setDriveBrakeMode(true);
    }

    public double getAzimuthSetpoint() {
        return this.desiredOptimizedAzimuthPosition;
    }
    public double getThrustEncoderVelocity() {
        return inputs.driveVelocityRadPerSec;
    }
    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Drivetrain/Module" + Integer.toString(this.id), inputs);
        
        double currentDistance = (WHEEL_DIAMETER * Math.PI)  *  inputs.drivePositionRad / (2*Math.PI);
        this.delta = currentDistance - this.lastDistance;
        this.lastDistance = currentDistance;

        if(Constants.getMode() == Mode.SIM) {
            double currentDistanceSimNoNoise = (WHEEL_DIAMETER * Math.PI)  *  inputs.drivePositionSimNoNoise / (2*Math.PI);
            this.deltaSimNoNoise = currentDistanceSimNoNoise - this.lastDistanceSimNoNoise; 
            this.lastDistanceSimNoNoise = currentDistanceSimNoNoise;
        }
       

        if(kPAzimuth.hasChanged(hashCode()) || kIAzimuth.hasChanged(hashCode()) || kDAzimuth.hasChanged(hashCode())) {
            turningPIDController.setP(kPAzimuth.get());
            turningPIDController.setI(kIAzimuth.get());
            turningPIDController.setD(kDAzimuth.get());
        }
        if(turnMaxpercent.hasChanged(hashCode())){
            turnMaxpercentLocal = turnMaxpercent.get();
        }
        
        double turnOutput;
       
        turnOutput = turningPIDController.calculate(inputs.turnAbsolutePositionDegrees, desiredState.angle.getDegrees());
        
        //Logger.recordOutput("Drive/Module" + Integer.toString(this.id) + "", id);
        io.setTurn(MathUtil.clamp(turnOutput, -turnMaxpercentLocal, turnMaxpercentLocal));
        Logger.recordOutput("Drivetrain/Module" + Integer.toString(this.id) + "/error",turningPIDController.getPositionError());
        Logger.recordOutput("Drivetrain/Module" + Integer.toString(this.id) + "/deltanonoise",this.deltaSimNoNoise);
        //Logger.recordOutput("Drive/Module" + Integer.toString(this.id) + "/setpoint",turningPIDController.getSetpoint().position);
        double angleErrorPenalty = Math.abs(Math.cos(desiredState.angle.minus(Rotation2d.fromDegrees(inputs.turnAbsolutePositionDegrees)).getRadians()));
        io.setDrive(desiredState.speedMetersPerSecond * angleErrorPenalty);
        // if(velAcc.hasChanged(hashCode()) || velMax.hasChanged(hashCode())){
        //     turningPIDController.setConstraints(new Constraints(velMax.get(),velAcc.get()));
        // }
    }
}

