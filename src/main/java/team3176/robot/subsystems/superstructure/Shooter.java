package team3176.robot.subsystems.superstructure;

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

  private final TunablePID pivotPIDController;
  private Rotation2d pivotSetpoint = new Rotation2d();

  private Shooter(ShooterIO io) {
    this.io = io;
    this.pivotPIDController = new TunablePID("shooter/pid", 0.0, 0.0, 0.0);
    pivotPIDController.setTolerance(Units.degreesToRadians(3.0));
  }

  public static Shooter getInstance() {
    if (instance == null) {
      if (Constants.getMode() != Mode.SIM) {
        instance = new Shooter(new ShooterIOFalcon() {});
      } else {
        instance = new Shooter(new ShooterIOSim() {});
      }
    }
    return instance;
  }

  /**
   * @param desiredAngle in degrees in Encoder Frame
   */
  private void PIDPositionPeriodic() {
    double pivotVoltage =
        pivotPIDController.calculate(inputs.pivotPosition.getRadians(), pivotSetpoint.getRadians());
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

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    pivotPIDController.checkParemeterUpdate();
    Logger.processInputs("Shooter", inputs);
    Logger.recordOutput("Shooter/desired", pivotSetpoint);
    // simSholder.setAngle(Rotation2d.fromDegrees(inputs.Position -90 -
    // SuperStructureConstants.ARM_SIM_OFFSET))

    // SmartDashboard.putNumber("Arm_Position", armEncoder.getAbsolutePosition());
    // SmartDashboard.putNumber("Arm_Position_Relative", armEncoder.getAbsolutePosition() -
    // SuperStructureConstants.ARM_ZERO_POS);

    Logger.recordOutput("Shooter/position_error", this.pivotPIDController.getPositionError());
    PIDPositionPeriodic();
  }
}
