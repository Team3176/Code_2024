package team3176.robot.commands.drivetrain;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import team3176.robot.subsystems.drivetrain.Drivetrain;
import team3176.robot.subsystems.vision.VisionCubeChase;
import edu.wpi.first.math.controller.PIDController;
import team3176.robot.subsystems.drivetrain.LimelightHelpers;
import team3176.robot.subsystems.superstructure.Claw;




public class CubeAlignAtPoint extends Command {
  private Drivetrain drivetrain = Drivetrain.getInstance();
  private VisionCubeChase visionCubeChase = VisionCubeChase.getInstance();
  private Claw claw = Claw.getInstance();


  //private DoubleSupplier forwardCommand;
  //private DoubleSupplier strafeCommand;
  //private DoubleSupplier spinCommand;
  private double forwardCommand;
  private double strafeCommand;
  private double splicingSpinCommand;
  private double smallNum;

  //PIDController txController = new PIDController(.01, 0, 0);
  //double tx;
  double tx; 
  double[] txArray = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  int arrayIdx;

  public CubeAlignAtPoint() {
    addRequirements(drivetrain);
    addRequirements(visionCubeChase);
    arrayIdx = 0;
  }

  @Override
  public void initialize() {
    //drivetrain.setCoastMode();
    //this.tx = visionCubeChase.getTx();
    double smallNum = Math.pow(10, -5);
  }

  @Override
  public void execute() {
    drivetrain.driveVelocity(new ChassisSpeeds(smallNum, smallNum, 1.0));
    tx = LimelightHelpers.getTX("limelight-three");
    txArray[arrayIdx] = tx;
    if (arrayIdx == (txArray.length + 1)) {arrayIdx = 0;} else {arrayIdx++;};
  }

  @Override
  public boolean isFinished() {
    boolean tvArrayTripwire = true;
    for (int idx = 0; idx < txArray.length; idx++) {
      tx = LimelightHelpers.getTX("limelight-three");
      txArray[idx] = tx;
    } 
    for (int idx = 0; idx < txArray.length; idx++) {
      tx = LimelightHelpers.getTX("limelight-three");
      if (txArray[idx] >= -19 && txArray[idx] <= +19) { 
        tvArrayTripwire = false; 
      };
    }
    if (tvArrayTripwire) { return true;} else {return false;}

  }

  @Override
  public void end(boolean interrupted) { 
   }
}