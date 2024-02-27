package team3176.robot.subsystems.superstructure.shooter;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import team3176.robot.Constants;
import team3176.robot.Constants.Mode;
import team3176.robot.FieldConstants;
import team3176.robot.subsystems.drivetrain.Drivetrain;
import team3176.robot.util.LoggedTunableNumber;
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
  public static final Translation3d shooterTranslation = new Translation3d(-0.01, 0.0, 0.4309);
  private final TunablePID pivotPIDController;
  private final LoggedTunableNumber aimAngle;
  private final LoggedTunableNumber flywheelUpperVelocity;
  private final LoggedTunableNumber flywheelLowerVelocity;
  private Rotation2d pivotSetpoint = new Rotation2d();
  private Rotation2d pivotOffSet = new Rotation2d();

  private Shooter(ShooterIO io) {
    this.io = io;
    this.pivotPIDController = new TunablePID("shooter/pid", 0.75, 0.0, 0.01);
    this.aimAngle = new LoggedTunableNumber("shooter/angle", 0);
    this.flywheelUpperVelocity = new LoggedTunableNumber("shooter/velocityUpper", 10.0);
    this.flywheelLowerVelocity = new LoggedTunableNumber("shooter/velocityLower", 10.0);
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
    if (pivotSetpoint.getDegrees() > 1.0) {
      pivotVoltage += 1;
    }
    pivotVoltage = MathUtil.clamp(pivotVoltage, -12, 12);
    io.setPivotVoltage(pivotVoltage);
  }

  @AutoLogOutput
  private Rotation2d getAimAngle() {
    Pose3d current =
        new Pose3d(Drivetrain.getInstance().getPose())
            .transformBy(new Transform3d(shooterTranslation, new Rotation3d()));
    Translation3d goal = FieldConstants.Speaker.centerSpeakerOpening;
    Translation3d diff = current.getTranslation().minus(goal);
    double z = diff.getZ();
    double distance = diff.toTranslation2d().getNorm();
    Rotation2d angle = Rotation2d.fromRadians(Math.atan2(z, distance));
    return Rotation2d.fromDegrees(aimAngle.get());
  }

  public Rotation2d getAngle() {
    return Rotation2d.fromRadians(inputs.pivotPosition.getRadians());
  }

  public void setUpperShooterVelocityVoltage(double d) {
    io.setWheelUpperVoltage(d);
  }

  public void setLowerShooterVelocityVoltage(double d) {
    io.setFlywheelVelocity(d);
  }

  public void setShooterStop() {
    io.setFlywheelVelocity(0);
  }

  public Command pivotSetPositionOnce(double angleInDegrees) {
    return this.runOnce(
            () -> {
              this.pivotSetpoint = Rotation2d.fromDegrees(angleInDegrees);
            })
        .withName("armSetPosition" + angleInDegrees);
  }

  public Command aim() {

    return this.runEnd(
        () -> {
          io.setFlywheelLowerVelocity(flywheelLowerVelocity.get());
          io.setFlywheelUpperVelocity(flywheelUpperVelocity.get());
          this.pivotSetpoint = getAimAngle();
        },
        () -> {
          io.setFlywheelVelocity(0);
          pivotSetpoint = new Rotation2d();
        });
  }

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

    Logger.recordOutput("Shooter/position_offset", inputs.pivotPosition.minus(pivotOffSet));
    PIDPositionPeriodic();
  }
}
