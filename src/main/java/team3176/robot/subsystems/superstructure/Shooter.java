package team3176.robot.subsystems.superstructure;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;
import team3176.robot.Constants;
import team3176.robot.Constants.Mode;
import team3176.robot.util.TunablePID;

public class Shooter extends SubsystemBase {
  private static Shooter instance;
  private final ShooterIO io;
  private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();
  private TalonFXConfiguration configsWheelPort = new TalonFXConfiguration();
  private TalonFXConfiguration configsWheelStarbrd = new TalonFXConfiguration();
  // Taken from CAD check later;
  public static final Rotation2d UPPER_LIMIT = Rotation2d.fromDegrees(54.46);
  public static final Rotation2d LOWER_LIMIT = Rotation2d.fromDegrees(13.4592);

  private final TunablePID pivotPIDController;
  private Rotation2d pivotSetpoint = new Rotation2d();
  private Rotation2d pivotOffSet = new Rotation2d();

  private Shooter(ShooterIO io) {
    this.io = io;
    this.pivotPIDController = new TunablePID("shooter/pid", 0.75, 0.0, 0.01);
    pivotPIDController.setTolerance(Units.degreesToRadians(3.0));
  }

  public static Shooter getInstance() {
    if (instance == null) {
      if (Constants.getMode() != Mode.SIM) {
        instance = new Shooter(new ShooterIOTalonSpark() {});

      } else {
        instance = new Shooter(new ShooterIOSim() {});
      }
    }
    return instance;
  }

  private void PIDPositionPeriodic() {
    Rotation2d positionAfterOffset = inputs.pivotPosition.minus(pivotOffSet);
    double pivotVoltage =
        pivotPIDController.calculate(positionAfterOffset.getRadians(), pivotSetpoint.getRadians());
    pivotVoltage = MathUtil.clamp(pivotVoltage, -12, 12);
    io.setPivotVoltage(pivotVoltage);
  }

  public Command pivotSetPositionOnce(double angleInDegrees) {
    return this.runOnce(
            () -> {
              this.pivotSetpoint = Rotation2d.fromDegrees(angleInDegrees);
            })
        .withName("armSetPosition" + angleInDegrees);
  }

  public Rotation2d getAngle() {
    return Rotation2d.fromRadians(inputs.pivotPosition.getRadians());
  }

  public void setUpperShooterVelocityVoltage(double d) {
    io.setWheelUpperVoltage(d);
  }

  public void setLowerShooterVelocityVoltage(double d) {
    io.setWheelLowerVoltage(d);
  }

  public void setLowerShooterVelocityVoltage2(double d) {
    io.setWheelLowerVoltage2(d);
  }

  public void setShooterStop() {
    io.setVelocityVoltage(0);
  }

  // public void setShooterPivotPercent(double d) {
  //   io.setPercentVoltage(d);
  // }

  // public void setShooterPivotPosition(int position) {

  //   io.setShooterPivotPID(position);
  // }

  // public void stopShooterPivotPosition() {
  //   io.setShooterPivotPID(0);
  // }

  // public void setShooterPivotVoltage() {
  //   io.setShooterPivotVoltage(0);
  // }

  public void debugger() {
    System.out.println(
        "BIG FLAG TO MAKE SURE YOU CAN SEE THIS CORRECTLY!! THIS IS FOR DEBUGGING!!! I WANT TO MAKE SURE YOU CAN SEE THIS!!!!");
  }

  // From Jonathan: I don't know if you still need this

  //   public void ShooterPIDVelocity(double velocity) {

  //     double desiredRotationsPerSecond = velocity * 50; // Go for plus/minus 10 rotations per
  // second

  //     // /* Use voltage velocity */
  //     // m_fx.setControl(m_voltageVelocity.withVelocity(desiredRotationsPerSecond));
  //     // return m_fx.getaver
  //     // }
  //     // else {
  //     // /* Disable the motor instead */
  //     // m_fx.setControl(m_brake);

  //   }

  //   public void ControllerTovVelocity(double JoystickButtonbuttonimput) {
  //     if (JoystickButtonbuttonimput < 0.1 && JoystickButtonbuttonimput > -0.1)
  //       JoystickButtonbuttonimput = 0;
  //     double velocity;
  //     velocity = JoystickButtonbuttonimput * 50;
  //   }

  public Command pivotVoltage(double volts) {
    return this.run(() -> io.setPivotVoltage(volts));
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    pivotPIDController.checkParemeterUpdate();
    Logger.processInputs("Shooter", inputs);
    if (inputs.lowerLimitSwitch) {
      pivotOffSet = inputs.pivotPosition;
    }
    Logger.recordOutput("Shooter/desired", pivotSetpoint);

    Logger.recordOutput("Shooter/position_error", this.pivotPIDController.getPositionError());
    PIDPositionPeriodic();
  }
}
