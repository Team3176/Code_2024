package team3176.robot.commands.drivetrain;

import java.util.HashMap;
import java.util.List;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import team3176.robot.subsystems.drivetrain.Drivetrain;
import team3176.robot.subsystems.superstructure.Superstructure;

public class PathPlannerAutoOld {
    Command auto;
    public PathPlannerAutoOld(String autoPathName, Command doBefore) {
        Drivetrain driveSubsystem = Drivetrain.getInstance();
        Superstructure superstructure = Superstructure.getInstance();
        // List<PathPlannerTrajectory> pathGroup = PathPlanner.loadPathGroup(autoPathName, 
        //         new PathConstraints(4.5,2.0));  //2.0, 1.5
        //System.out.println("length" + pathGroup.size());
        // This is just an example event map. It would be better to have a constant, global event map
        // in your code that will be used by all path following commands.
        HashMap<String, Command> eventMap = new HashMap<>();
        eventMap.put("scoreHighFirst", superstructure.scoreGamePieceAuto());
        eventMap.put("scoreHigh", superstructure.scoreGamePieceAuto());
        eventMap.put("autoBalance", new AutoBalance().andThen(driveSubsystem.swerveDefenseCommand()).finallyDo((b) -> {
            driveSubsystem.driveVelocity(new ChassisSpeeds(0, 0, 0));
        }));
        eventMap.put("groundCube",superstructure.groundCube().withTimeout(3));
        eventMap.put("poopCube",superstructure.poopCube().withTimeout(.7));
        eventMap.put("CubeAlignAtPoint", new CubeAlignAtPoint());
        eventMap.put("CubeChaseOn", new CubeChaseAutonOn());
        eventMap.put("CubeChaseOff", new CubeChaseOff());
        eventMap.put("delay", new WaitCommand(10.0));
        // eventMap.put("intakeDown", new IntakeDown());
        // Create the AutoBuilder. This only needs to be created once when robot code starts, not every time you want to create an auto command. A good place to put this is in RobotContainer along with your subsystems.
        // SwerveAutoBuilder autoBuilder = new SwerveAutoBuilder(
        //     driveSubsystem::getPose,
        //     driveSubsystem::resetPose,
        //     DrivetrainConstants.DRIVE_KINEMATICS,
        //     new PIDConstants(5.0,0.0,0.0),
        //     new PIDConstants(3.5,0.0,0.0),
        //     driveSubsystem::setModuleStates,
        //     eventMap,
        //     true, driveSubsystem);
        // if (doBefore != null){
        //     auto = doBefore.andThen(autoBuilder.fullAuto(pathGroup));
        // } else {
        //     auto = autoBuilder.fullAuto(pathGroup);
        // }
        
    }
    public PathPlannerAutoOld(String autoPathName) {
        this(autoPathName,null);
    }
    public Command getauto(){
        //TODO: Implement new pathplanner auto
        return new WaitCommand(1.0);
    }

}
