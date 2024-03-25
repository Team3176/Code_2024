package team3176.robot.subsystems.superstructure.shooter;

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
import team3176.robot.Constants.RobotType;
import team3176.robot.FieldConstants;
import team3176.robot.subsystems.drivetrain.Drivetrain;
import team3176.robot.util.AllianceFlipUtil;
import team3176.robot.util.LoggedTunableNumber;
import team3176.robot.util.TunablePID;

public class Shooter extends SubsystemBase {
  private static Shooter instance;
  private final ShooterIO io;
  private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();
  // Taken from CAD check later;
  public static final Rotation2d UPPER_LIMIT = Rotation2d.fromDegrees(54.46);
  public static final Rotation2d LOWER_LIMIT = Rotation2d.fromDegrees(13.4592);
  public static final Translation3d shooterTranslation = new Translation3d(-0.01, 0.0, 0.4309);
  public static final double FLYWHEEL_IDLE = 50;

  private final TunablePID pivotPIDController;
  private final LoggedTunableNumber aimAngle;
  private final LoggedTunableNumber flywheelUpperVelocity;
  private final LoggedTunableNumber flywheelLowerVelocity;
  private final LoggedTunableNumber forwardPivotVoltageOffset;
  private Rotation2d pivotSetpoint = new Rotation2d();
  private Rotation2d pivotOffSet = new Rotation2d();
  // private InterpolatingDoubleTreeMap shooterFlywheelLookup;

  private Shooter(ShooterIO io) {
    this.io = io;
    this.pivotPIDController = new TunablePID("shooter/pid", 4.0, 0.25, 0.00);
    pivotPIDController.setIntegratorRange(-0.5, 0.5);
    pivotPIDController.setTolerance(Units.degreesToRadians(0.5));
    this.aimAngle = new LoggedTunableNumber("shooter/angle", 30);
    this.flywheelUpperVelocity = new LoggedTunableNumber("shooter/velocityUpper", 60.0);
    this.flywheelLowerVelocity = new LoggedTunableNumber("shooter/velocityLower", 60.0);
    this.forwardPivotVoltageOffset = new LoggedTunableNumber("shooter/pivotOffset", 0.55);
    // shooterFlywheelLookup.put(1.0, 60.0);
    // shooterFlywheelLookup.put(3.2, 100.0);
  }

  public static Shooter getInstance() {
    if (instance == null) {
      if (Constants.getMode() != Mode.SIM && Constants.getRobot() != RobotType.ROBOT_DEFENSE) {
        instance = new Shooter(new ShooterIOTalonSpark() {});

      } else {
        instance = new Shooter(new ShooterIOSim() {});
      }
    }
    return instance;
  }

  private void PIDPositionPeriodic() {
    double pivotVoltage =
        pivotPIDController.calculate(getPosition().getRadians(), pivotSetpoint.getRadians());
    if (pivotSetpoint.getDegrees() > 1.0) {
      pivotVoltage +=
          forwardPivotVoltageOffset.get()
              * Math.cos(getPosition().getRadians() + Units.degreesToRadians(13));
    }
    pivotVoltage = MathUtil.clamp(pivotVoltage, -0.25, 12);
    io.setPivotVoltage(pivotVoltage);
  }

  @AutoLogOutput
  public Rotation2d getPosition() {
    return inputs.pivotPosition.minus(pivotOffSet);
  }

  @AutoLogOutput
  private Rotation2d getAimAngle() {
    // Pose3d current =
    //     new Pose3d(Drivetrain.getInstance().getPose())
    //         .transformBy(new Transform3d(shooterTranslation, new Rotation3d()));
    // Translation3d goal = FieldConstants.Speaker.centerSpeakerOpening;
    // Translation3d diff = current.getTranslation().minus(goal);
    // double z = diff.getZ();
    // double distance = diff.toTranslation2d().getNorm();
    // Logger.recordOutput("shooter/distance", diff.toTranslation2d().getNorm());
    // Rotation2d angle = Rotation2d.fromRadians(Math.atan2(z, distance));

    return Rotation2d.fromDegrees(aimAngle.get());
  }

  @AutoLogOutput
  public double getDistance() {
    Pose3d current =
        new Pose3d(Drivetrain.getInstance().getPose())
            .transformBy(new Transform3d(shooterTranslation, new Rotation3d()));
    Translation3d goal = AllianceFlipUtil.apply(FieldConstants.Speaker.centerSpeakerOpening);
    Translation3d diff = current.getTranslation().minus(goal);
    double distance = diff.toTranslation2d().getNorm();
    Logger.recordOutput("Shooter/podium", distance < 2.6 && distance > 2.37);
    return distance;
  }

  public Rotation2d getAngle() {
    return Rotation2d.fromRadians(inputs.pivotPosition.getRadians());
  }

  @AutoLogOutput
  public boolean isAtSpeed() {
    double closeValue = 0.1;
    return (Math.abs(inputs.lowerWheelError) < closeValue
        && Math.abs(inputs.upperWheelError) < closeValue);
  }

  @AutoLogOutput
  public boolean isAtAngle() {
    return pivotPIDController.atSetpoint();
  }

  public boolean readyToShoot() {
    return isAtSpeed() && isAtAngle();
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
          io.setFlywheelVelocity(FLYWHEEL_IDLE);
          pivotSetpoint = new Rotation2d();
        });
  }

  public Command aimLookup() {

    return this.runEnd(
        () -> {
          // io.setFlywheelLowerVelocity(shooterFlywheelLookup.get(getDistance()));
          // io.setFlywheelUpperVelocity(shooterFlywheelLookup.get(getDistance()));
          this.pivotSetpoint = getAimAngle();
        },
        () -> {
          io.setFlywheelVelocity(FLYWHEEL_IDLE);
          pivotSetpoint = new Rotation2d();
        });
  }

  public Command aim(double upper, double lower, double angleDegrees) {

    return this.runEnd(
        () -> {
          io.setFlywheelLowerVelocity(upper);
          io.setFlywheelUpperVelocity(lower);
          this.pivotSetpoint = Rotation2d.fromDegrees(angleDegrees);
        },
        () -> {
          io.setFlywheelVelocity(FLYWHEEL_IDLE);
          pivotSetpoint = new Rotation2d();
        });
  }

  public Command stopFlywheel() {
    return this.runOnce(() -> io.setFlywheelVelocity(0.0));
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

    Logger.recordOutput("Shooter/position-error", this.pivotPIDController.getPositionError());
    PIDPositionPeriodic();
  }
}
