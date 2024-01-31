package team3176.robot.subsystems.superstructure;


import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import team3176.robot.constants.SuperStructureConstants;

public class Superstructure extends SubsystemBase {
    private static Superstructure instance;
    private Elevator elevator;
    private Intake intake;
    private Transfer transfer;
    private Shooter shooter;
    public Superstructure() {
        elevator = Elevator.getInstance();
        intake = Intake.getInstance();
        //shooter = Shooter.getInstance();
        //transfer = Transfer  .getInstance();
    }
    public static Superstructure getInstance() {
        if (instance == null){instance = new Superstructure();}
        return instance;
    }
    
    public Command moveElevator(double position){
        return this.run(() -> elevator.setElevatorMotor(position));
      }


    public Command moveIntake(double velocity){
        return this.run(() -> intake.setIntakeMotor(velocity));
    }

    
}
