package team3176.robot.subsystems.superstructure;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;
import team3176.robot.Constants;
import team3176.robot.Constants.Mode;
import team3176.robot.constants.SuperStructureConstants;

public class Transfer extends SubsystemBase {
  private static Transfer instance;
  private final TransferIO io;
  private final TransferIOInputsAutoLogged inputs = new TransferIOInputsAutoLogged();

  private double armSetpointAngleRaw;
  private final PIDController turningPIDController;

  public enum States {
    OPEN_LOOP,
    CLOSED_LOOP
  }

  private States currentState = States.OPEN_LOOP;
  private Mechanism2d mech = new Mechanism2d(3, 3);
  private MechanismRoot2d root = mech.getRoot("armRoot", 0.0, 0.42);
  private MechanismLigament2d armSholder =
      root.append(new MechanismLigament2d("armLigament", .4, 90));
  // private MechanismLigament2d simSholder = root.append(new
  // MechanismLigament2d("simLigament",3,90,10,new Color8Bit(Color.kAqua)));
  private MechanismLigament2d armElbow =
      armSholder.append(new MechanismLigament2d("armELigament", 0.5, 90));

  private Transfer(TransferIO io) {
    this.io = io;
    this.turningPIDController =
        new PIDController(
            SuperStructureConstants.TRANSFER_kP,
            SuperStructureConstants.TRANSFER_kI,
            SuperStructureConstants.TRANSFER_kD);
    SmartDashboard.putNumber("Arm_kp", SuperStructureConstants.TRANSFER_kP);
    SmartDashboard.putNumber("Arm_Kg", SuperStructureConstants.TRANSFER_kg);
    SmartDashboard.putNumber("arm_angle", 0.0);
    SmartDashboard.putNumber("arm_height", 1.18);
    setArmPidPosMode();
  }

  public void setCoastMode() {
    io.setCoastMode(true);
  }

  public void setBrakeMode() {
    io.setCoastMode(false);
  }

  private void setArmPidPosMode() {

    this.turningPIDController.setTolerance(SuperStructureConstants.TRANSFER_TOLERANCE);
    // this.m_turningPIDController.enableContinuousInput()
    this.turningPIDController.reset();
    this.turningPIDController.setP(SuperStructureConstants.TRANSFER_kP);
    this.turningPIDController.setI(SuperStructureConstants.TRANSFER_kI);
    this.turningPIDController.setD(SuperStructureConstants.TRANSFER_kD);
    // this.m_turningPIDController.enableContinuousInput(0, 360);
  }

  public static Transfer getInstance() {
    if (instance == null) {
      if (Constants.getMode() != Mode.SIM) {
        instance = new Transfer(new TransferIOSpark() {});
      } else {
        instance = new Transfer(new TransferIOSim() {});
      }
    }
    return instance;
  }

  /**
   * @param desiredAngle in degrees in Encoder Frame
   */
  private void setPIDPosition(double desiredAngle) {
    // need to double check these values

    // double physicsAngle = (desiredAngle - SuperStructureConstants.ARM_CARRY_POS);

    // kg is the scalar representing the percent power needed to hold the arm at 90 degrees away
    // from the robot
    // double kg = SmartDashboard.getNumber("Arm_Kg", SuperStructureConstants.ARM_kg);
    // kp set as the fraction of control effort / error to cause control effort
    // for example .4 output is generated by a 40 degree error
    double kp = SmartDashboard.getNumber("Arm_kp", SuperStructureConstants.TRANSFER_kP);
    turningPIDController.setP(kp);
    double feedForward = 0.0; // kg * physicsAngle/SuperStructureConstants.ARM_HIGH_POS;
    /*         if (inputs.Position < SuperStructureConstants.ARM_MID_POS + 10){
                feedForward =0.0;
            } else if (desiredAngle < SuperStructureConstants.ARM_ZERO_POS+5) {
                feedForward = -.2;
            }
    */
    double turnOutput = turningPIDController.calculate(inputs.Position, desiredAngle);
    turnOutput = MathUtil.clamp(turnOutput, -1, 1);
    io.set(turnOutput + feedForward);
    SmartDashboard.putNumber("Arm_Output", turnOutput + feedForward);
    SmartDashboard.putNumber("Arm Feed Forward", feedForward);
  }

  public void armAnalogUp() {
    this.currentState = States.OPEN_LOOP;
    io.set(SuperStructureConstants.ARM_OUTPUT_POWER);
  }

  public void armAnalogDown() {
    this.currentState = States.OPEN_LOOP;
    io.set(-SuperStructureConstants.ARM_OUTPUT_POWER);
    // System.out.println("Arm Analog Down");
    // System.out.println("Arm_Abs_Position: " + armEncoder.getAbsolutePosition());
    // System.out.println("Arm_Rel_Position: " + armEncoder.getPosition());
  }

  public void idle() {
    io.set(0.0);
  }

  public void fineTune(double delta) {
    this.currentState = States.CLOSED_LOOP;
    this.armSetpointAngleRaw -= delta * 0.5;
  }

  public double getArmPosition() {
    return inputs.Position;
  }

  public boolean isArmAtPosition() {
    return Math.abs(this.turningPIDController.getPositionError()) < 7;
  }
  /**
   * to be used for trajectory following without disrupting other commands
   *
   * @param setpointAngle
   */
  public void setAngleSetpoint(double setpointAngle) {
    this.currentState = States.CLOSED_LOOP;
    this.armSetpointAngleRaw = setpointAngle;
  }
  /*
   * Commands
   */
  public Command armSetPosition(double angleInDegrees) {
    Logger.recordOutput("Arm/armSetPosition", angleInDegrees);
    return this.run(() -> setPIDPosition(angleInDegrees))
        .withName("armSetPosition" + angleInDegrees);
  }

  public Command armSetPositionBlocking(double angleInDegrees) {
    return new FunctionalCommand(
            () -> {
              this.currentState = States.CLOSED_LOOP;
              this.armSetpointAngleRaw = angleInDegrees;
            },
            () -> {},
            b -> {},
            this::isArmAtPosition,
            this)
        .withName("armsetPositionBlocking");
  }

  public Command armSetPositionOnce(double angleInDegrees) {
    return this.runOnce(
            () -> {
              this.currentState = States.CLOSED_LOOP;
              this.armSetpointAngleRaw = angleInDegrees;
            })
        .withName("armSetPosition" + angleInDegrees);
  }

  public Command armFineTune(DoubleSupplier angleDeltaCommand) {
    return this.run(() -> fineTune(angleDeltaCommand.getAsDouble())).withName("armFineTune");
  }

  public Command armAnalogUpCommand() {
    return this.runEnd(this::armAnalogUp, this::idle);
  }

  public Command armAnalogDownCommand() {
    return this.runEnd(this::armAnalogDown, this::idle);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Arm", inputs);
    armSholder.setAngle(
        Rotation2d.fromDegrees(inputs.Position - SuperStructureConstants.TRANSFER_ZERO_POS - 190));
    // simSholder.setAngle(Rotation2d.fromDegrees(inputs.Position -90 -
    // SuperStructureConstants.ARM_SIM_OFFSET));
    Logger.recordOutput("Arm/mech2d", mech);
    double angle = SmartDashboard.getNumber("arm_angle", 0.0);
    double height = SmartDashboard.getNumber("arm_height", 1.18);
    double angle_for_vis = (-inputs.Position) + 240 + 130;
    Pose3d shoulder =
        new Pose3d(
            0.199155,
            0.0,
            1.18,
            new Rotation3d(0.0, Units.degreesToRadians(angle_for_vis - 130), 0.0));
    Pose3d shoulder_offset =
        new Pose3d(
            0.199155, 0.0, 1.18, new Rotation3d(0.0, Units.degreesToRadians(angle_for_vis), 0.0));
    Pose3d elbow =
        shoulder_offset.transformBy(
            new Transform3d(
                new Translation3d(0.5, 0.0, 0.0),
                new Rotation3d(
                    0.0,
                    Units.degreesToRadians(
                        -((50
                                + 120
                                    * (SuperStructureConstants.TRANSFER_CARRY_POS
                                        - (inputs.Position))
                                    / (SuperStructureConstants.TRANSFER_CARRY_POS
                                        - SuperStructureConstants.TRANSFER_ZERO_POS)))
                            + 130
                            + 240),
                    0.0)));
    // Pose3d elbow = new Pose3d(-0.106, 0.0, 0.779603, new Rotation3d(0.0,
    // Units.degreesToRadians(angle), 0.0));
    Pose3d[] arm_joints = {shoulder, elbow};
    Logger.recordOutput("Transfer/visual", arm_joints);
    // SmartDashboard.putNumber("Arm_Position", armEncoder.getAbsolutePosition());
    // SmartDashboard.putNumber("Arm_Position_Relative", armEncoder.getAbsolutePosition() -
    // SuperStructureConstants.ARM_ZERO_POS);
    if (this.currentState == States.CLOSED_LOOP) {
      this.armSetpointAngleRaw =
          MathUtil.clamp(
              this.armSetpointAngleRaw,
              SuperStructureConstants.TRANSFER_ZERO_POS,
              SuperStructureConstants.TRANSFER_PICKUP_POS);
      Logger.recordOutput("Arm/position_error", this.turningPIDController.getPositionError());
      setPIDPosition(armSetpointAngleRaw);
    }
  }
}
