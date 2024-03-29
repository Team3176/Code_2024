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
  // public static final double FLYWHEEL_IDLE = 20;

  private final TunablePID pivotPIDController;
  private final LoggedTunableNumber aimAngle;
  private final LoggedTunableNumber flywheelLeftVelocity;
  private final LoggedTunableNumber flywheelRightVelocity;
  private final LoggedTunableNumber forwardPivotVoltageOffset;
  private final LoggedTunableNumber forwardPivotScale;
  private final LoggedTunableNumber flywheelIdle;
  private Rotation2d pivotSetpoint = new Rotation2d();
  private Rotation2d pivotOffSet = new Rotation2d();
  private InterpolatingDoubleTreeMap shooterFlywheelLookupLeft;
  private InterpolatingDoubleTreeMap shooterFlywheelLookupRight;
  private InterpolatingDoubleTreeMap pivotLookup;

  private Shooter(ShooterIO io) {
    this.io = io;
    this.pivotPIDController = new TunablePID("shooter/pid", 4.0, 0.6, 0.00);
    pivotPIDController.setIntegratorRange(-0.5, 0.5);
    pivotPIDController.setIZone(Units.degreesToRadians(5));
    pivotPIDController.setTolerance(Units.degreesToRadians(0.5));
    this.aimAngle = new LoggedTunableNumber("shooter/angle", 16.5);
    this.flywheelLeftVelocity = new LoggedTunableNumber("shooter/velocityLeft", 90.0);
    this.flywheelRightVelocity = new LoggedTunableNumber("shooter/velocityRight", 40.0);
    this.forwardPivotVoltageOffset = new LoggedTunableNumber("shooter/pivotOffset", 1.0);
    this.forwardPivotScale = new LoggedTunableNumber("shooter/pivotScale", 1.0);
    this.flywheelIdle = new LoggedTunableNumber("shooter/idleVel", 20);
    pivotLookup = new InterpolatingDoubleTreeMap();
    pivotLookup.put(1.07, 35.0);
    pivotLookup.put(1.62, 25.0);
    pivotLookup.put(1.97, 22.0);
    pivotLookup.put(2.3, 19.5);
    pivotLookup.put(2.7, 18.0);
    pivotLookup.put(2.69, 18.0);
    pivotLookup.put(2.92, 16.5);
    pivotLookup.put(2.98, 16.5);

    shooterFlywheelLookupLeft = new InterpolatingDoubleTreeMap();
    shooterFlywheelLookupRight = new InterpolatingDoubleTreeMap();
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
    boolean do_on_rio = true;
    double ffvolts =
            forwardPivotVoltageOffset.get()* Math.cos(getPosition().getRadians() * forwardPivotScale.get() + Units.degreesToRadians(6));
    if(do_on_rio) {
      double pivotVoltage =
        pivotPIDController.calculate(getPosition().getRadians(), pivotSetpoint.getRadians());
      pivotVoltage = MathUtil.clamp(pivotVoltage, -1.0, 1.0);
      Logger.recordOutput("Shooter/pidRawOut", pivotVoltage);
      if (pivotSetpoint.getDegrees() > 1.0) {
        pivotVoltage += ffvolts;
      }
      Logger.recordOutput(
          "Shooter/feedforwardVolts",
          forwardPivotVoltageOffset.get()
              * Math.cos(getPosition().getRadians() + Units.degreesToRadians(13)));

      pivotVoltage = MathUtil.clamp(pivotVoltage, -0.25, 3);
      Logger.recordOutput("shooter/velocityError", pivotPIDController.getVelocityError());
      Logger.recordOutput("Shooter/commandedVolts", pivotVoltage);
      io.setPivotVoltage(pivotVoltage);
    } else {
      io.setPivotPosition(pivotSetpoint.minus(pivotOffSet), ffvolts);
    }
    
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
    return (Math.abs(inputs.rightWheelError) < closeValue
        && Math.abs(inputs.leftWheelError) < closeValue);
  }

  @AutoLogOutput
  public boolean isAtAngle() {
    return pivotPIDController.atSetpoint() || inputs.pivotPosition.getDegrees() > 26.0;
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
          if (getDistance() < 1.1) {
            io.setFlywheelRightVelocity(80.0);
            io.setFlywheelLeftVelocity(80.0);
          } else {
            io.setFlywheelRightVelocity(flywheelRightVelocity.get());
            io.setFlywheelLeftVelocity(flywheelLeftVelocity.get());
          }

          this.pivotSetpoint = getAimAngle();
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
    pivotPIDController.checkParemeterUpdate();
    Logger.processInputs("Shooter", inputs);
    if (inputs.lowerLimitSwitch) {
      pivotOffSet = inputs.pivotPosition;
    }
    Logger.recordOutput("shooter/offset", pivotOffSet);
    Logger.recordOutput("Shooter/desired", pivotSetpoint);

    Logger.recordOutput("Shooter/position-error", this.pivotPIDController.getPositionError());
    PIDPositionPeriodic();
  }
}
