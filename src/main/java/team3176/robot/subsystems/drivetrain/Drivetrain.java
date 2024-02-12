// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team3176.robot.subsystems.drivetrain;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.pathfinding.Pathfinding;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PathPlannerLogging;
import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.*;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.ArrayList;
import java.util.List;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import team3176.robot.Constants;
import team3176.robot.Constants.Mode;
import team3176.robot.FieldConstants;
import team3176.robot.constants.Hardwaremap;
import team3176.robot.constants.SwervePodHardwareID;
import team3176.robot.subsystems.drivetrain.GyroIO.GyroIOInputs;
import team3176.robot.subsystems.vision.PhotonVisionSystem;
import team3176.robot.util.AllianceFlipUtil;
import team3176.robot.util.LocalADStarAK;
import team3176.robot.util.TunablePID;
import team3176.robot.util.swerve.ModuleLimits;
import team3176.robot.util.swerve.SwerveSetpoint;
import team3176.robot.util.swerve.SwerveSetpointGenerator;

public class Drivetrain extends SubsystemBase {

  public static final double MAX_WHEEL_SPEED = 4.2;
  public static final double EBOT_LENGTH_IN_METERS_2023 = Units.inchesToMeters(30 - 6);
  public static final double EBOT_WIDTH_IN_METERS_2023 = Units.inchesToMeters(30 - 6);
  public static final double LENGTH = EBOT_LENGTH_IN_METERS_2023;
  public static final double WIDTH = EBOT_WIDTH_IN_METERS_2023;

  private static Drivetrain instance;
  private SwerveDriveOdometry odom;
  private SwerveDrivePoseEstimator poseEstimator;

  // private Controller controller = Controller.getInstance();
  // private Vision m_Vision = Vision.getInstance();
  public enum coordType {
    FIELD_CENTRIC,
    ROBOT_CENTRIC
  }

  // private PowerDistribution PDH = new PowerDistribution();
  // PowerDistribution(PowerManagementConstants.PDP_CAN_ID, ModuleType.kCTRE);

  private ArrayList<SwervePod> pods;

  Rotation2d fieldAngleOffset = Rotation2d.fromDegrees(0.0);
  public static final Translation2d[] SwerveModuleTranslations = {
    new Translation2d(LENGTH / 2.0, -WIDTH / 2.0), // FR where +x=forward and -y=starboard
    new Translation2d(LENGTH / 2.0, WIDTH / 2.0), // FL where +x=forward and +y=port
    new Translation2d(-LENGTH / 2.0, WIDTH / 2.0), // BL where -x=backward(aft) and +y=port
    new Translation2d(-LENGTH / 2.0, -WIDTH / 2.0) // BR where -x=backward(aft) and -y=starboard
  };
  public static final SwerveDriveKinematics kinematics =
      new SwerveDriveKinematics(SwerveModuleTranslations);
  // TODO: Update values
  public static ModuleLimits moduleLimits =
      new ModuleLimits(MAX_WHEEL_SPEED, 8.0, Units.degreesToRadians(700.0));
  private SwervePod podFR;
  private SwervePod podFL;
  private SwervePod podBL;
  private SwervePod podBR;

  Rotation2d wheelOnlyHeading = new Rotation2d();
  private final GyroIO io;
  private GyroIOInputs inputs;
  public Pose3d visionPose3d;
  SimNoNoiseOdom simNoNoiseOdom;
  SwerveSetpointGenerator setpointGenerator;
  SwerveSetpoint prevSetpoint =
      new SwerveSetpoint(
          new ChassisSpeeds(),
          new SwerveModuleState[] {
            new SwerveModuleState(),
            new SwerveModuleState(),
            new SwerveModuleState(),
            new SwerveModuleState()
          });
  private ChassisSpeeds desiredSpeeds;
  private TunablePID orientationPID;
  // private final DrivetrainIOInputs inputs = new DrivetrainIOInputs();

  private Drivetrain(GyroIO io) {
    this.io = io;
    inputs = new GyroIOInputs();

    // check for duplicates
    assert (!SwervePodHardwareID.check_duplicates_all(
        Hardwaremap.FR, Hardwaremap.FL, Hardwaremap.BR, Hardwaremap.BL));
    // Instantiate pods
    if (Constants.getMode() != Mode.REPLAY) {
      switch (Constants.getRobot()) {
        case ROBOT_2024C:
          System.out.println("[init] normal swervePods");
          podFR =
              new SwervePod(
                  0, new SwervePodIOFalconSpark(Hardwaremap.FR, Hardwaremap.STEER_FR_CID));
          podFL =
              new SwervePod(
                  1, new SwervePodIOFalconSpark(Hardwaremap.FL, Hardwaremap.STEER_FL_CID));
          podBL =
              new SwervePod(
                  2, new SwervePodIOFalconSpark(Hardwaremap.BL, Hardwaremap.STEER_BL_CID));
          podBR =
              new SwervePod(
                  3, new SwervePodIOFalconSpark(Hardwaremap.BR, Hardwaremap.STEER_BR_CID));
          break;
        case CTRL_BOARD:
        case ROBOT_SIMBOT:
          System.out.println("[init] simulated swervePods");
          podFR = new SwervePod(0, new SwervePodIOSim(0));
          podFL = new SwervePod(1, new SwervePodIOSim(1));
          podBL = new SwervePod(2, new SwervePodIOSim(2));
          podBR = new SwervePod(3, new SwervePodIOSim(3));
          simNoNoiseOdom = new SimNoNoiseOdom(new ArrayList<>(List.of(podFR, podFL, podBL, podBR)));
          break;
        default:
          break;
      }
    } else {
      podFR = new SwervePod(0, new SwervePodIO() {});
      podFL = new SwervePod(1, new SwervePodIO() {});
      podBL = new SwervePod(2, new SwervePodIO() {});
      podBR = new SwervePod(3, new SwervePodIO() {});
    }

    // Instantiate array list then add instantiated pods to list
    pods = new ArrayList<>();
    pods.add(podFR);
    pods.add(podFL);
    pods.add(podBL);
    pods.add(podBR);

    visionPose3d = new Pose3d();
    odom =
        new SwerveDriveOdometry(
            kinematics,
            this.getSensorYaw(),
            new SwerveModulePosition[] {
              podFR.getPosition(), podFL.getPosition(), podBL.getPosition(), podBR.getPosition()
            },
            new Pose2d(0.0, 0.0, new Rotation2d()));
    poseEstimator =
        new SwerveDrivePoseEstimator(
            kinematics, getSensorYaw(), getSwerveModulePositions(), odom.getPoseMeters());

    AutoBuilder.configureHolonomic(
        this::getPose,
        this::resetPose,
        () -> kinematics.toChassisSpeeds(getModuleStates()),
        this::driveVelocity,
        new HolonomicPathFollowerConfig(4.0, LENGTH, new ReplanningConfig()),
        () -> {
          // Boolean supplier that controls when the path will be mirrored for the red alliance
          // This will flip the path being followed to the red side of the field.
          // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

          var alliance = DriverStation.getAlliance();
          if (alliance.isPresent()) {
            return alliance.get() == DriverStation.Alliance.Red;
          }
          return false;
        },
        this);
    Pathfinding.setPathfinder(new LocalADStarAK());
    PathPlannerLogging.setLogActivePathCallback(
        (activePath) -> {
          Logger.recordOutput(
              "Odometry/Trajectory", activePath.toArray(new Pose2d[activePath.size()]));
        });
    PathPlannerLogging.setLogTargetPoseCallback(
        (targetPose) -> {
          Logger.recordOutput("Odometry/TrajectorySetpoint", targetPose);
        });
    setpointGenerator =
        SwerveSetpointGenerator.builder()
            .kinematics(kinematics)
            .moduleLocations(SwerveModuleTranslations)
            .build();
    orientationPID = new TunablePID("Drivetrain/orientationPID", 1.0, 0.0, 0.0);
    orientationPID.enableContinuousInput(-Math.PI, Math.PI);
  }

  // Prevents more than one instance of drivetrian
  public static Drivetrain getInstance() {
    if (instance == null) {
      if (Constants.getMode() != Mode.SIM) {
        instance = new Drivetrain(new GyroIONavX());
      } else {
        instance = new Drivetrain(new GyroIO() {});
      }
    }
    return instance;
  }

  public void driveVelocity(ChassisSpeeds speeds) {
    ChassisSpeeds discreteSpeeds = ChassisSpeeds.discretize(speeds, 0.02);
    Logger.recordOutput("Drivetrain/speedsRaw", discreteSpeeds);
    Logger.recordOutput(
        "SwerveSetpoints/BeforePoofs", kinematics.toSwerveModuleStates(discreteSpeeds));
    // SwerveSetpoint output =
    //     setpointGenerator.generateSetpoint(moduleLimits, prevSetpoint, discreteSpeeds, 0.02);
    // SwerveModuleState[] podStates = output.moduleStates();
    // prevSetpoint = output;
    SwerveModuleState[] podStates = kinematics.toSwerveModuleStates(discreteSpeeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(podStates, MAX_WHEEL_SPEED);
    SwerveModuleState[] optimizedStates = new SwerveModuleState[4];
    for (int idx = 0; idx < (pods.size()); idx++) {
      optimizedStates[idx] = pods.get(idx).setModule(podStates[idx]);
    }
    // Logger.recordOutput(
    //     "Drivetrain/speedsOptimized",
    //     new double[] {
    //       output.chassisSpeeds().vxMetersPerSecond,
    //       output.chassisSpeeds().vyMetersPerSecond,
    //       output.chassisSpeeds().omegaRadiansPerSecond
    //     });
    Logger.recordOutput("SwerveSetpoints/Setpoints", podStates);
    Logger.recordOutput("SwerveSetpoints/SetpointsOptimized", optimizedStates);
  }

  public void driveVelocityFieldCentric(ChassisSpeeds speeds) {
    Rotation2d fieldOffset = this.getPose().getRotation();
    if (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red) {
      fieldOffset.plus(Rotation2d.fromDegrees(180));
    }
    speeds = ChassisSpeeds.fromFieldRelativeSpeeds(speeds, fieldOffset);
    driveVelocity(speeds);
  }

  @AutoLogOutput
  public Pose2d getPose() {
    return poseEstimator.getEstimatedPosition();
  }

  @AutoLogOutput
  public Pose2d getPoseOdom() {
    return odom.getPoseMeters();
  }

  @AutoLogOutput
  public Pose2d getSimNoNoisePose() {
    if (Constants.getMode() == Mode.SIM) {
      return simNoNoiseOdom.getPoseTrue();
    }
    return new Pose2d();
  }

  public void addVisionMeasurement(Pose3d p, double time, Matrix<N3, N1> cov) {
    visionPose3d = p;
    poseEstimator.addVisionMeasurement(p.toPose2d(), time, cov);
  }

  public void resetPose(Pose2d pose) {

    if (Constants.getMode() == Mode.SIM) {
      simNoNoiseOdom.resetSimPose(pose);
    }
    wheelOnlyHeading = pose.getRotation();
    odom.resetPosition(getSensorYaw(), getSwerveModulePositions(), pose);
    poseEstimator.resetPosition(getSensorYaw(), getSwerveModulePositions(), pose);
  }

  /** Returns the module states (turn angles and drive velocitoes) for all of the modules. */
  @AutoLogOutput(key = "SwerveSetpoints/ModuleStates")
  private SwerveModuleState[] getModuleStates() {
    SwerveModuleState[] states = new SwerveModuleState[4];
    for (int i = 0; i < 4; i++) {
      states[i] = pods.get(i).getState();
    }
    return states;
  }
  /**
   * @return returns the chassis yaw wrapped between -pi and pi
   */
  public Rotation2d getPoseYawWrapped() {
    // its ugly but rotation2d is continuos but I imagine most of our applications
    // we want it bounded between -pi and pi
    return Rotation2d.fromRadians(MathUtil.angleModulus(getPose().getRotation().getRadians()));
  }

  /**
   * The unbounded angle
   *
   * @return Rotation2d of the yaw
   */
  @AutoLogOutput
  protected Rotation2d getSensorYaw() {
    if (Constants.getMode() == Mode.SIM) {
      if (this.odom == null || this.poseEstimator == null) {
        return new Rotation2d();
      }
      return simNoNoiseOdom.getPoseTrue().getRotation();
    }
    return inputs.rotation2d;
  }

  public Rotation2d getChassisYaw() {
    return getPose().getRotation();
  }

  /**
   * @return navx pitch -180 to 180 around the X axis of the Navx
   */
  public double getChassisPitch() {
    return inputs.pitch;
  }

  /**
   * @return navx roll -180 to 180 around the X axis of the Navx
   */
  public double getChassisRoll() {
    return inputs.roll;
  }

  public void resetFieldOrientation() {
    // do not need to invert because the navx rotation2D call returns a NWU
    // coordsys!
    // this.FieldAngleOffset = m_NavX.getRotation2d();
    Rotation2d redOrBlueZero = new Rotation2d();
    // if (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red) {
    //   redOrBlueZero.plus(Rotation2d.fromDegrees(180));
    // }
    resetPose(new Pose2d(getPose().getTranslation(), redOrBlueZero));
  }

  public SwerveModulePosition[] getSwerveModulePositions() {
    return new SwerveModulePosition[] {
      podFR.getPosition(), podFL.getPosition(), podBL.getPosition(), podBR.getPosition()
    };
  }

  public void setCoastMode() {
    for (int idx = 0; idx < (pods.size()); idx++) {
      pods.get(idx).setThrustCoast();
    }
  }

  public void setBrakeMode() {
    for (int idx = 0; idx < (pods.size()); idx++) {
      pods.get(idx).setThrustBrake();
    }
  }

  //TODO
  private Rotation2d getAimAngle() {
    return new Rotation2d();
  }

  public Command swerveDefenseCommand() {
    return this.runOnce(
        () -> {
          kinematics.resetHeadings(
              Rotation2d.fromDegrees(-45),
              Rotation2d.fromDegrees(45),
              Rotation2d.fromDegrees(-45),
              Rotation2d.fromDegrees(45));
          this.driveVelocity(new ChassisSpeeds());
        });
  }

  public Command swerveDrivePercent(
      DoubleSupplier forward, DoubleSupplier strafe, DoubleSupplier spin, boolean isFieldCentric) {
    return this.run(
        () -> {
          ChassisSpeeds speeds =
              new ChassisSpeeds(
                  forward.getAsDouble() * MAX_WHEEL_SPEED,
                  strafe.getAsDouble() * MAX_WHEEL_SPEED,
                  spin.getAsDouble() * MAX_WHEEL_SPEED);
          if (isFieldCentric) {
            driveVelocityFieldCentric(speeds);
          } else {
            driveVelocity(speeds);
          }
        });
  }

  public Command swerveDrivePercent(
      DoubleSupplier forward, DoubleSupplier strafe, DoubleSupplier spin) {
    return swerveDrivePercent(forward, strafe, spin, true);
  }

  /*
   * Does Deadbanding and such within drivetrain
   */
  public Command swerveDriveJoysticks(
      DoubleSupplier forward,
      DoubleSupplier strafe,
      DoubleSupplier spin,
      Boolean isFieldCentric,
      Supplier<Rotation2d> angle) {
    return this.run(
        () -> {
          double DEADBAND = 0.05;
          double linearMagnitude =
              MathUtil.applyDeadband(
                  Math.hypot(forward.getAsDouble(), strafe.getAsDouble()), DEADBAND);
          Rotation2d linearDirection = new Rotation2d(forward.getAsDouble(), strafe.getAsDouble());
          double omega = MathUtil.applyDeadband(spin.getAsDouble(), DEADBAND);

          // Square values
          linearMagnitude = MathUtil.clamp(linearMagnitude * linearMagnitude, -1.0, 1.0) * 0.9;
          omega = Math.copySign(omega * omega, omega);
          // Calcaulate new linear velocity
          Translation2d linearVelocity =
              new Pose2d(new Translation2d(), linearDirection)
                  .transformBy(new Transform2d(linearMagnitude, 0.0, new Rotation2d()))
                  .getTranslation();
          if (DriverStation.getAlliance().isPresent()
              && DriverStation.getAlliance().get() == DriverStation.Alliance.Red
              && isFieldCentric) {
            linearVelocity = linearVelocity.rotateBy(Rotation2d.fromRadians(Math.PI));
          }

          ChassisSpeeds speeds =
              new ChassisSpeeds(
                  linearVelocity.getX() * MAX_WHEEL_SPEED,
                  linearVelocity.getY() * MAX_WHEEL_SPEED,
                  omega * 10.5);
          if (angle != null) {
            //TODO: DRIVE AIM
            speeds.omegaRadiansPerSecond = 0.0;
            driveVelocityFieldCentric(speeds);
          } else {
            if (isFieldCentric) {
              driveVelocityFieldCentric(speeds);
            } else {
              driveVelocity(speeds);
            }
          }
        });
  }

  public Command swerveDriveJoysticks(
      DoubleSupplier forward, DoubleSupplier strafe, DoubleSupplier spin) {
    return swerveDriveJoysticks(forward, strafe, spin, true, null);
  }

  public Command swerveDriveJoysticks(
      DoubleSupplier forward, DoubleSupplier strafe, DoubleSupplier spin, boolean field) {
    return swerveDriveJoysticks(forward, strafe, spin, field, null);
  }

  public Command driveAndAim(DoubleSupplier x, DoubleSupplier y) {
    return swerveDriveJoysticks(
        x,
        y,
        () -> 0.0,
        true,
        this::getAimAngle);
  }

  // TODO: Pathplanner pathfinding
  // complete the following method that returns a command from AutoBuilder that goes to the point
  // given
  public Command goToPoint(int x, int y) {
    Pose2d targetPose = new Pose2d(x, y, Rotation2d.fromDegrees(180));
    PathConstraints constraints =
        new PathConstraints(3.0, 1.0, Units.degreesToRadians(540), Units.degreesToRadians(720));
    return AutoBuilder.pathfindToPose(targetPose, constraints);
    // Command pathfindingCommand = new PathfindHolonomic(targetPose, constraints,
    // AutoBuilder::getPose 0.0, AutoBuilder::getVelocity, AutoBuilder:: get, 0.0, null);
    // Replace with your command

  }

  public Command chaseNote() {
    return this.run(
        () -> {
          if (PhotonVisionSystem.getInstance().seeNote) {
            double yawError = -8 - PhotonVisionSystem.getInstance().noteYaw;
            double pitchError = -20 - PhotonVisionSystem.getInstance().notePitch;
            ChassisSpeeds speed =
                new ChassisSpeeds(
                    MathUtil.clamp(pitchError * (1 / 10.0), -1.0, 1.0), 0, yawError * (1 / 20.0));
            driveVelocity(speed);
          } else {
            if (PhotonVisionSystem.getInstance().notePitch < -12) {
              driveVelocity(new ChassisSpeeds(-0.7, 0.0, 0.0));
            }
            driveVelocity(new ChassisSpeeds());
          }
        });
  }

  public Command stop() {
    return new InstantCommand(() -> driveVelocity(new ChassisSpeeds()));
  }

  private void resetPoseToVision() {
    // call resetPose() and pass in visionPose3d
    resetPose(visionPose3d.toPose2d());
    // use the .toPose2d() method to convert a 3d pose into a 2d pose

  }
  // TODO: @Liam 11/27
  public Command resetPoseToVisionCommand() {
    // we want to use a instant command

    // InstantCommand takes in a function reference either () -> { your code here}
    // or a reference to an existing function this::function_name
    // here we want to use the resetPoseToVision function you created earlier
    // use the this::function_name pattern and construct a new InstantCommand to be returned

    // Replace with your command
    return new InstantCommand(this::resetPoseToVision);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    orientationPID.checkParemeterUpdate();
    if (!DriverStation.isEnabled()) {
      prevSetpoint = new SwerveSetpoint(new ChassisSpeeds(), getModuleStates());
      driveVelocity(new ChassisSpeeds());
    }
    Logger.processInputs("Drivetrain/gyro", inputs);
    SwerveModulePosition[] deltas = new SwerveModulePosition[4];

    for (int i = 0; i < pods.size(); i++) {
      deltas[i] = pods.get(i).getDelta();
    }
    Twist2d twist = kinematics.toTwist2d(deltas);
    wheelOnlyHeading = getPoseOdom().exp(twist).getRotation();
    // update encoders
    this.poseEstimator.update(getSensorYaw(), getSwerveModulePositions());
    this.odom.update(getSensorYaw(), getSwerveModulePositions());

    if (Constants.getMode() == Mode.SIM) {
      simNoNoiseOdom.update();
    }

    SmartDashboard.putNumber("NavYaw", getPoseYawWrapped().getDegrees());
  }
}
