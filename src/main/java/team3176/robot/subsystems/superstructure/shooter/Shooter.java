package team3176.robot.subsystems.superstructure.shooter;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
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
  public static final double LOOKAHEAD_SEC = 0.5;
  // public static final double FLYWHEEL_IDLE = 20;
  private double lastDistance = 0;
  private double futureDistance = 0;
  private double lastTimestamp;

  private final TunablePID pivotPIDController;
  private final LoggedTunableNumber aimAngle;
  private final LoggedTunableNumber flywheelLeftVelocity;
  private final LoggedTunableNumber flywheelRightVelocity;
  private final LoggedTunableNumber forwardPivotVoltageOffset;
  private final LoggedTunableNumber flywheelIdle;

  private double rightWheelSetpoint = 0.0;
  private double leftWheelSetpoint = 0.0;
  private Rotation2d pivotSetpoint = new Rotation2d();
  private Rotation2d pivotOffSet = new Rotation2d();
  private Rotation2d pivotTopPosition;
  private InterpolatingDoubleTreeMap shooterFlywheelLookupLeft;
  private InterpolatingDoubleTreeMap shooterFlywheelLookupRight;
  private InterpolatingDoubleTreeMap pivotLookup;
  private InterpolatingDoubleTreeMap pivotFeedForward = new InterpolatingDoubleTreeMap();
  public boolean isHomed = false;

  private Shooter(ShooterIO io) {
    this.io = io;
    this.pivotTopPosition = new Rotation2d(Units.degreesToRadians(39));
    this.pivotPIDController = new TunablePID("shooter/pid", 8.0, 0.0, 0.5);
    pivotPIDController.setIntegratorRange(-0.5, 0.5);
    pivotPIDController.setIZone(Units.degreesToRadians(6));
    pivotPIDController.setTolerance(Units.degreesToRadians(1.0));
    this.aimAngle = new LoggedTunableNumber("shooter/angle", 40);
    this.flywheelLeftVelocity = new LoggedTunableNumber("shooter/velocityLeft", 90.0);
    this.flywheelRightVelocity = new LoggedTunableNumber("shooter/velocityRight", 40.0);
    this.forwardPivotVoltageOffset = new LoggedTunableNumber("shooter/pivotOffset", 0.7);
    this.flywheelIdle = new LoggedTunableNumber("shooter/idleVel", 20);
    pivotLookup = new InterpolatingDoubleTreeMap();
    pivotLookup.put(1.07, 38.0);
    pivotLookup.put(1.62, 25.0);
    pivotLookup.put(2.0, 22.0);
    pivotLookup.put(2.33, 22.0);
    pivotLookup.put(2.36, 21.0);
    pivotLookup.put(2.45, 19.0);
    pivotLookup.put(2.48, 19.0);
    pivotLookup.put(2.52, 19.0);
    pivotLookup.put(2.54, 19.0);
    pivotLookup.put(2.57, 19.0);
    pivotLookup.put(2.61, 19.0);
    pivotLookup.put(2.63, 19.0);
    pivotLookup.put(2.67, 19.0);
    pivotLookup.put(2.69, 19.0);
    pivotLookup.put(2.72, 17.8);
    pivotLookup.put(2.78, 17.8);
    pivotLookup.put(2.85, 17.0);
    pivotLookup.put(2.93, 17.0);
    pivotLookup.put(3.03, 17.0);
    pivotLookup.put(3.11, 17.0);
    pivotLookup.put(3.21, 16.0);
    pivotLookup.put(3.3, 15.5);
    pivotLookup.put(3.38, 15.0);
    pivotLookup.put(3.47, 14.5);
    pivotLookup.put(3.58, 13.75);
    pivotLookup.put(3.7, 13.75);
    pivotLookup.put(3.94, 13.5);

    shooterFlywheelLookupLeft = new InterpolatingDoubleTreeMap();
    shooterFlywheelLookupRight = new InterpolatingDoubleTreeMap();

    io.setFlywheelVelocity(0.0);
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
    double clamp = 1.0; // 0.3 * Math.cos(getPosition().getRadians()) + 0.4;
    pivotVoltage = MathUtil.clamp(pivotVoltage, -clamp, clamp);
    if (pivotSetpoint.getDegrees() > 1.0) {
      pivotVoltage += forwardPivotVoltageOffset.get();
      // * Math.cos(
      //     getPosition().getRadians()
      //         * 0.8); // pivotFeedForward.get(getPosition().getDegrees());
    }
    pivotVoltage = MathUtil.clamp(pivotVoltage, -1.0, 2.5);
    if (pivotSetpoint.getDegrees() < 1.0) {
      pivotVoltage = 0.0;
    }
    io.setPivotVoltage(pivotVoltage);
  }

  private void manualFallbackPeriodic() {
    double pivotVoltage = 0.0;
    if (pivotSetpoint.getDegrees() > 1.0) {
      pivotVoltage = 2.0;
      // if we are at the top hold there
      if (inputs.upperLimitSwitch) {
        pivotVoltage = 1.0;
      }
    }

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

    return Rotation2d.fromDegrees(pivotLookup.get(getDistance()));
  }

  @AutoLogOutput
  private Rotation2d getAimAngleFuture() {
    return Rotation2d.fromDegrees(pivotLookup.get(getDistanceFuture()));
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

  @AutoLogOutput
  public double getDistanceFuture() {
    Pose3d current =
        new Pose3d(Drivetrain.getInstance().getPoseFuture(LOOKAHEAD_SEC))
            .transformBy(new Transform3d(shooterTranslation, new Rotation3d()));
    Translation3d goal = AllianceFlipUtil.apply(FieldConstants.Speaker.centerSpeakerOpening);
    Translation3d diff = current.getTranslation().minus(goal);
    double distance = diff.toTranslation2d().getNorm();
    return distance;
  }

  public Rotation2d getAngle() {
    return Rotation2d.fromRadians(inputs.pivotPosition.getRadians());
  }

  @AutoLogOutput
  public boolean isAtSpeed() {
    double closeValue = 10.0;
    return (Math.abs(rightWheelError()) < closeValue && Math.abs(leftWheelError()) < closeValue)
        && inputs.leftWheelReference > flywheelIdle.get();
  }

  @AutoLogOutput
  private double rightWheelError() {
    return inputs.rightWheelReference
        - Units.radiansToRotations(inputs.wheelRightVelocityRadPerSec);
  }

  @AutoLogOutput
  private double leftWheelError() {
    return inputs.leftWheelReference - Units.radiansToRotations(inputs.wheelLeftVelocityRadPerSec);
  }

  @AutoLogOutput
  public boolean isAtAngle() {
    return pivotPIDController.atSetpoint() || inputs.pivotPosition.getDegrees() > 33.0;
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
          io.setFlywheelRightVelocity(flywheelRightVelocity.get());
          io.setFlywheelLeftVelocity(flywheelLeftVelocity.get());
          this.pivotSetpoint = Rotation2d.fromDegrees(aimAngle.get());
        },
        () -> {
          io.setFlywheelVelocity(flywheelIdle.get());
          pivotSetpoint = new Rotation2d();
        });
  }

  public Command aimLookup() {

    return this.runEnd(
        () -> {
          //          if (getDistance() < 1.1) {
          if (false) {
            io.setFlywheelRightVelocity(80.0);
            io.setFlywheelLeftVelocity(80.0);
          } else {
            io.setFlywheelRightVelocity(flywheelRightVelocity.get());
            io.setFlywheelLeftVelocity(flywheelLeftVelocity.get());
          }

          this.pivotSetpoint = getAimAngleFuture();
        },
        () -> {
          io.setFlywheelVelocity(flywheelIdle.get());
          pivotSetpoint = new Rotation2d();
        });
  }

  public Command aim(double left, double right, double angleDegrees) {

    return this.runEnd(
        () -> {
          io.setFlywheelRightVelocity(left);
          io.setFlywheelLeftVelocity(right);
          this.pivotSetpoint = Rotation2d.fromDegrees(angleDegrees);
        },
        () -> {
          io.setFlywheelVelocity(flywheelIdle.get());
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
    //    this.futureDistance =
    //        getDistance()
    //            * ((getDistance() - this.lastDistance) / this.lastDistance)
    //            * (Timer.getFPGATimestamp() - this.lastTimestamp);
    //    this.lastDistance = getDistance();
    //    this.lastTimestamp = Timer.getFPGATimestamp();
    pivotPIDController.checkParemeterUpdate();
    Logger.processInputs("Shooter", inputs);
    if (inputs.upperLimitSwitch && !isHomed) {
      pivotOffSet = inputs.pivotPosition.minus(pivotTopPosition);
      isHomed = true;
    }
    Logger.recordOutput("Shooter/desired", pivotSetpoint);

    Logger.recordOutput("Shooter/position-error", this.pivotPIDController.getPositionError());
    if (isHomed) {
      PIDPositionPeriodic();
    } else {
      // run in manual mode doing bang bang control based off limit switch
      manualFallbackPeriodic();
    }
  }
}
