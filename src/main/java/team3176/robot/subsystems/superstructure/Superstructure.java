package team3176.robot.subsystems.superstructure;


import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import team3176.robot.constants.SuperStructureConstants;

public class Superstructure extends SubsystemBase {
    private static Superstructure instance;
    private Arm arm;
    private Claw claw;
    private IntakeCone intakeCone;
    private IntakeCube intakeCube;
    public Superstructure() {
        arm = Arm.getInstance();
        claw = Claw.getInstance();
        intakeCone = IntakeCone.getInstance();
        intakeCube = IntakeCube.getInstance();
    }
    public static Superstructure getInstance() {
        if (instance == null){instance = new Superstructure();}
        return instance;
    }
    
    

    public enum GamePiece {CUBE, CONE, NONE}
    

    public Command groundCube() {
        return new WaitCommand(1.0);
        //return new IntakeGroundCube().andThen(this.prepareCarry()).withName("groundCube");
    }

    public Command clawIntakeCube()
    {
        return new InstantCommand(() -> claw.intakeGamePiece(GamePiece.CUBE)).withName("clawIntakeCube");
    }

    
    public Command poopCube() {
        return new ParallelCommandGroup(arm.armSetPosition(SuperStructureConstants.ARM_ZERO_POS),intakeCube.spit(),claw.scoreGamePiece());
    }
    
    public Command scoreGamePieceAuto() {
        return claw.determineGamePiece()
                .andThen(arm.armSetPositionBlocking(SuperStructureConstants.ARM_HIGH_POS).withTimeout(1.5)
                    .andThen(claw.scoreGamePiece())
                    .andThen(this.prepareCarry())).withName("scoreGamePieceAuto");
    }
    public Command scoreFirstGamePieceAuto() {
        return claw.determineGamePiece()
                .andThen(intakeCone.extendAndFreeSpin().withTimeout(1.0)
                .alongWith(arm.armSetPositionBlocking(SuperStructureConstants.ARM_HIGH_POS).withTimeout(3.0)
                .andThen(new WaitCommand(0.5))
                    .andThen(claw.scoreGamePiece().withTimeout(1.0))
                    .andThen(this.prepareCarry()))).withName("scoreFirstGamePieceAuto");
    }
    public Command scoreGamePieceHigh()
    {
        return claw.determineGamePiece()
                .andThen(arm.armSetPositionBlocking(SuperStructureConstants.ARM_HIGH_POS).withTimeout(3.0))
                .andThen(new WaitCommand(0.5))
                .andThen(claw.scoreGamePiece().withTimeout(1.0))
                .andThen(this.prepareCarry()).withName("scoreGamePieceHigh");
    }
    public Command scoreCubeLow() {
        return arm.armSetPosition(SuperStructureConstants.ARM_ZERO_POS)
        .andThen(new WaitCommand(0.5))
        .andThen(claw.scoreGamePiece())
        .andThen(this.prepareCarry()).withName("scoreCubeLow");
    }
    public Command scoreGamePieceLowAuto()
    {
        return claw.determineGamePiece()
                .andThen(arm.armSetPositionBlocking(SuperStructureConstants.ARM_CATCH_POS).withTimeout(3.0))
                .andThen(new WaitCommand(0.5))
                .andThen(claw.scoreGamePiece().withTimeout(1.0))
                .andThen(this.prepareCarry()).withName("scoreGamePieceLowAuto");
    }
    public Command intakeCubeHumanPlayer() {
        return new ParallelCommandGroup(claw.intakeCube(), arm.armSetPositionOnce(SuperStructureConstants.ARM_HIGH_POS))
        .andThen(arm.armSetPositionOnce(SuperStructureConstants.ARM_CARRY_POS)).withName("intakeCubeHumanPlayer");
    }

    public Command intakeConeHumanPlayer() {
        return new ParallelCommandGroup(claw.intakeCone(), arm.armSetPositionOnce(SuperStructureConstants.ARM_HIGH_POS))
        .andThen(arm.armSetPositionOnce(SuperStructureConstants.ARM_CARRY_POS)).withName("intakeConeHumanPlayer");
    }
    
    public Command preparePoop() {
        return arm.armSetPositionOnce(SuperStructureConstants.ARM_ZERO_POS);
    }
    public Command prepareCarry() {
        return arm.armSetPositionOnce(SuperStructureConstants.ARM_CARRY_POS);
    }
    public Command prepareCatch() {
        return arm.armSetPositionOnce(SuperStructureConstants.ARM_CATCH_POS);
    }
    public Command prepareScoreMid() {
        return arm.armSetPositionOnce(SuperStructureConstants.ARM_MID_POS);
    }
    public Command prepareScoreHigh() {
        return arm.armSetPositionOnce(SuperStructureConstants.ARM_HIGH_POS);
    }
    
}
