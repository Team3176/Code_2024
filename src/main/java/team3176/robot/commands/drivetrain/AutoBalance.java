package team3176.robot.commands.drivetrain;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import team3176.robot.subsystems.drivetrain.Drivetrain;

public class AutoBalance extends Command {
  private Drivetrain drivetrain;
  // private boolean isDone = false;
  // private int numBalanced = 0;
  public AutoBalance() {
    drivetrain = Drivetrain.getInstance();
    addRequirements(drivetrain);
  }

  @Override
  public void initialize() {
    drivetrain.setBrakeMode();
  }

  @Override
  public void execute() {
    // double Kp = 0.1;

    // Bang Bang controller!
    double forward;
    double deadbandDegrees = 8;
    SmartDashboard.putNumber("pitch", drivetrain.getChassisPitch());
    // if(m_Drivetrain.getChassisPitch() > 0 + deadbandDegrees) {
    //     forward = 0.37 * Math.pow(.96,num_balanced);
    // } else if(m_Drivetrain.getChassisPitch() < 0 - deadbandDegrees) {
    //     forward = -0.37 * Math.pow(.96,num_balanced);
    // } else if(Math.abs(m_Drivetrain.getChassisPitch()) < 2){
    //     num_balanced ++;
    // }
    // P loop option
    if (Math.abs(drivetrain.getChassisPitch()) > 0 + deadbandDegrees) {
      forward = 0.03 * drivetrain.getChassisPitch();
      forward = MathUtil.clamp(forward, -0.5, 0.5);
    } else {
      forward = 0.0;
    }

    drivetrain.driveVelocity(new ChassisSpeeds(forward, 0.0, 0.0));
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return Timer.getMatchTime() < 0.5;
  }
}
