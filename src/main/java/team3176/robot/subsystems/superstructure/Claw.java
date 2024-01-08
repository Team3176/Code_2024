package team3176.robot.subsystems.superstructure;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import team3176.robot.Constants;
import team3176.robot.Constants.Mode;
import team3176.robot.constants.SuperStructureConstants;
import team3176.robot.subsystems.superstructure.Superstructure.GamePiece;
import team3176.robot.util.DelayedBoolean;
import team3176.robot.subsystems.superstructure.ClawIO.ClawIOInputs;
import org.littletonrobotics.junction.Logger;

public class Claw extends SubsystemBase {
    private static Claw instance;
    private final ClawIO io;
    private final ClawIOInputs inputs = new ClawIOInputs();
    private GamePiece currentGamePiece = GamePiece.CONE;
    private Claw(ClawIO io) {
        this.io = io;
    }

    //states now implemented as functions
    
    public void intake() {
        //System.out.println("m_Claw.intake()");
        if(currentGamePiece == GamePiece.CUBE) {
            io.setClawMotor(SuperStructureConstants.CLAW_OUTPUT_POWER_CUBE,SuperStructureConstants.CLAW_CURRENT_LIMIT_A);
         }
         else if(currentGamePiece == GamePiece.CONE) {
             io.setClawMotor(-SuperStructureConstants.CLAW_OUTPUT_POWER_CONE,SuperStructureConstants.CLAW_CURRENT_LIMIT_A);
         }
    }
    
    public void hold() {
        //System.out.println("m_Claw.hold()");
        if(currentGamePiece == GamePiece.CUBE) {
            io.setClawMotor(SuperStructureConstants.CLAW_HOLD_POWER,SuperStructureConstants.CLAW_HOLD_CURRENT_LIMIT_A);
            //idle(); 
        } else {
            io.setClawMotor(-SuperStructureConstants.CLAW_HOLD_POWER * SuperStructureConstants.CLAW_HOLD_CONE_FACTOR,SuperStructureConstants.CLAW_HOLD_CURRENT_LIMIT_A);
        }
    }

    public void score() {
        //System.out.println("m_Claw.score()");
        if(currentGamePiece == GamePiece.CUBE) {
            io.setClawMotor(-SuperStructureConstants.CLAW_OUTPUT_POWER_CUBE,SuperStructureConstants.CLAW_CURRENT_LIMIT_A);
        }
        else if(currentGamePiece == GamePiece.CONE) {
            io.setClawMotor(SuperStructureConstants.CLAW_OUTPUT_POWER_CONE,SuperStructureConstants.CLAW_CURRENT_LIMIT_A);
        }
    }
    public void idle() {
        //System.out.println("m_Claw.idle()");
        io.setClawMotor(0, 0);
    }

    public void setCurrentGamePiece(GamePiece piece) {
        //System.out.println("m_Claw.setCurrentGamePiece() to " + piece);
        this.currentGamePiece = piece;
    }

    public boolean isEmpty() {
        return getIsLinebreakOne() || getIsLinebreakTwo();
    }

    public static Claw getInstance()
    {
        if (instance == null ) {
            if(Constants.getMode() == Mode.REAL) {
                instance = new Claw(new ClawIOSpark());
            } else {
                instance = new Claw(new ClawIO() {});
            }
            
          }
        return instance;
    }

    /**
     * to be called with a whileTrue trigger binding
     */
    public Command intakeGamePiece(GamePiece piece) {
        return this.startEnd(() ->  {this.currentGamePiece = piece; intake();},() -> hold());
    }
    /**
     *  scores game piece then returns to idle state
     * @return to be called with a whileTrue or onTrue trigger binding
     */
    public Command scoreGamePiece() {  
        return this.run(() ->  {score(); this.currentGamePiece = GamePiece.NONE;})
                    .until(() -> this.isEmpty())
                    .andThen(new WaitCommand(0.7))
                    .andThen(this.runOnce(()->idle())).withTimeout(2.0).finallyDo((b)->idle()).withName("scoreGamepiece");
    }
    public Command scoreGamePieceTeleop() {
        return this.runEnd(() ->  {score(); this.currentGamePiece = GamePiece.NONE;},() -> idle());
    }
    //more examples of command composition and why its awesome!!
    public Command intakeCone() {
        return this.intakeGamePiece(GamePiece.CONE).until(new DelayedBoolean(0.5,() -> !getIsLinebreakTwo())::get).withName("intakeCone");
    }
    public Command intakeCube() {
        return this.intakeGamePiece(GamePiece.CUBE).until(new DelayedBoolean(0.5,() -> !getIsLinebreakOne())::get ).withName("intakeCube");
    }
    public Command determineGamePiece() {
        return this.runOnce( () -> {
            if(this.getIsLinebreakOne()) {
                this.currentGamePiece = GamePiece.CONE;
                hold();
            } else if(this.getIsLinebreakTwo()) {
                this.currentGamePiece = GamePiece.CUBE;
                hold();
            }
        }).withName("determineGamePiece");
    }
    public Command idleCommand() {
        return this.runOnce(this::idle);
    }



    @Override
  public void periodic() {
    // This method will be called once per scheduler run
    io.updateInputs(inputs);
    Logger.getInstance().processInputs("Claw", inputs);
    Logger.recordOutput("Claw/Velocity", getVelocity());
    Logger.recordOutput("Claw/LinebreakOne", getIsLinebreakOne());
    Logger.recordOutput("Claw/LinebreakTwo", getIsLinebreakTwo());
    // Code stating if something is in the Intake
    SmartDashboard.putBoolean("linebreakOne",getIsLinebreakOne());
    SmartDashboard.putBoolean("linebreakTwo",getIsLinebreakTwo());
    // SmartDashboard.putBoolean("isExtended", isExtended);

   }

   public double getVelocity()
   {
    return inputs.velocity;
   }

   public boolean getIsLinebreakOne()
   {
    return inputs.isLinebreakOne;
   }

   public boolean getIsLinebreakTwo()
   {
    return inputs.isLinebreakTwo;
   }
}
