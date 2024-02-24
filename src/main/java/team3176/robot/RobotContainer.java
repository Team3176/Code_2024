// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team3176.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import team3176.robot.commands.drivetrain.*;
import team3176.robot.constants.Hardwaremap;
import team3176.robot.subsystems.RobotState;
import team3176.robot.subsystems.Visualization;
import team3176.robot.subsystems.controller.Controller;
import team3176.robot.subsystems.superstructure.*;
import team3176.robot.subsystems.vision.PhotonVisionSystem;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...

  private final Controller controller;
  private PowerDistribution pdh;

  // is this why we don't have a compressor? private final Compressor m_Compressor
  // private Drivetrain drivetrain;
  private final RobotState robotState;
  private final Superstructure superstructure;
  private Climb climb;
  private PhotonVisionSystem vision;
  private Visualization visualization;
  private LoggedDashboardChooser<Command> autonChooser;
  private Command choosenAutonomousCommand = new WaitCommand(1.0);
  private Alliance currentAlliance = Alliance.Blue;
  private Visualization vis;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    controller = Controller.getInstance();
    /*
    if (Constants.getMode() == Mode.SIM) {
      drivetrain = Drivetrain.getInstance();
    }
    */
    superstructure = Superstructure.getInstance();
    robotState = RobotState.getInstance();
    climb = Climb.getInstance();
    // visualization = new Visualization();
    if (Constants.VISION_CONNECTED) {
      vision = PhotonVisionSystem.getInstance();
    }

    pdh = new PowerDistribution(Hardwaremap.PDH_CID, ModuleType.kRev);
    /*
        drivetrain.setDefaultCommand(
            drivetrain
                .swerveDriveJoysticks(
                    () -> controller.getForward(),
                    () -> controller.getStrafe(),
                    () -> controller.getSpin())
                .withName("default drive"));
        if (Constants.getMode() == Mode.SIM) {
          drivetrain.setDefaultCommand(
              drivetrain
                  .swerveDriveJoysticks(
                      () -> controller.getForward(),
                      () -> controller.getStrafe(),
                      () -> controller.getSpin(),
                      false)
                  .withName("default drive"));
        }
        NamedCommands.registerCommand(
            "shoot", new WaitCommand(0.5).alongWith(new PrintCommand("shoot")).withName("shooting"));
        // NamedCommands.registerCommand(
        //     "intake",
        //     intake
        //         .runIntake(-1.0)
        //         .withTimeout(0.5)
        //         .alongWith(new PrintCommand("intake"))
        //         .withName("intaking"));

        autonChooser = new LoggedDashboardChooser<>("autonChoice", AutoBuilder.buildAutoChooser());

        SmartDashboard.putData("Auton Choice", autonChooser.getSendableChooser());
    */
    configureBindings();
  }

  private void configureBindings() {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    /*
        // m_Controller.getTransStick_Button1().onFalse(new InstantCommand(() ->
        // m_Drivetrain.setTurbo(false), m_Drivetrain));
        // controller.transStick.button(2).whileTrue(drivetrain.pathfind("shoot"));
        // controller.transStick.button(3).whileTrue(drivetrain.pathfind("pickup"));
        controller.transStick.button(5).onTrue(drivetrain.resetPoseToVisionCommand());
        controller
            .transStick
            .button(10)
            .whileTrue(
                new InstantCommand(drivetrain::setBrakeMode)
                    .andThen(drivetrain.swerveDefenseCommand())
                    .withName("swerveDefense"));
        // m_Controller.getTransStick_Button10()
        //    .onFalse(new InstantCommand(() -> m_Drivetrain.setDriveMode(driveMode.DRIVE),
        // m_Drivetrain));

    // m_Controller.getTransStick_Button1().onFalse(new InstantCommand(() ->
    // m_Drivetrain.setTurbo(false), m_Drivetrain));
    /* TODO pathplanner-finding link button 2 on the transStick to the goToPoint.
      use the whileTrue so if the button is released the command is cancelled
      pass in a new Pose2d object for the point (2.0,2.0) you can pass a blank new Rotation2d() as the orientation
    */
    // controller.transStick.button(1).whileTrue(intake.runIntake(-0.6));

    // controller
    //     .transStick
    //     .button(3)
    //     .whileTrue(drivetrain.chaseNote().alongWith(intake.runIntake(-0.6)));

    /*
    controller.transStick.button(2).whileTrue(drivetrain.goToPoint(2, 2));

    controller.transStick.button(5).onTrue(drivetrain.resetPoseToVisionCommand());
    controller
        .transStick
        .button(10)
        .whileTrue(
            new InstantCommand(drivetrain::setBrakeMode)
                .andThen(drivetrain.swerveDefenseCommand())
                .withName("swerveDefense"));
    controller
        .rotStick
        .button(8)
        .whileTrue(new InstantCommand(drivetrain::resetFieldOrientation, drivetrain));

    controller
        .rotStick
        .button(3)
        .whileTrue(
            new InstantCommand(drivetrain::setBrakeMode)
                .andThen(drivetrain.swerveDefenseCommand())
                .withName("setBrakeMode"));

    controller
        .rotStick
        .button(8)
        .whileTrue(new InstantCommand(drivetrain::resetFieldOrientation, drivetrain));

    controller
        .rotStick
        .button(8)
        .whileTrue(new InstantCommand(drivetrain::resetFieldOrientation, drivetrain));
    */
    controller
        .operator
        .b()
        .whileTrue(superstructure.movePivotDown(-.25))
        .onFalse(superstructure.stopPivot());

    controller
        .operator
        .a()
        .whileTrue(superstructure.moveElevator(.5))
        .onFalse(superstructure.stopElevator());

    controller
        .operator
        .y()
        .whileTrue(superstructure.positiveIntake(50))
        .onFalse(superstructure.stopIntake());

    controller
        .operator
        .x()
        .whileTrue(superstructure.movePivotUp(.25))
        .onFalse(superstructure.stopPivot());
    // TODO: uncomment when done with voltage
    /*     controller
    .operator
    .leftBumper()
    .onTrue(superstructure.moveLeftClimb(45))
    .onFalse(superstructure.moveLeftClimb(0)); */

    controller
        .operator
        .leftBumper()
        .whileTrue(superstructure.setLeftVoltage(3))
        .onFalse(superstructure.setLeftVoltage(0));

    // controller.operator.leftBumper().onTrue(climb.leftGoToPosition(.5));

    // .whileTrue(superstructure.moveLeftClimb(() -> controller.getXboxJoyLeft()));
    // TODO: uncomment when done with voltage
    /* controller
    .operator
    .rightBumper()
    .whileTrue(superstructure.moveRightClimb(.5).andThen(superstructure.moveRightClimb(0))); */

    controller
        .operator
        .rightBumper()
        .whileTrue(superstructure.setRightVoltage(3))
        .onFalse(superstructure.setRightVoltage(0));

    // controller.operator.a().onTrue(superstructure.moveElevator(.5));
    // controller.operator.y().onTrue(superstructure.positiveIntake(50));
    // controller
    //    .transStick
    //    .button(1)
    //    .onTrue(Shooter.getInstance().pivotSetPositionOnce(50))
    //    .onFalse(Shooter.getInstance().pivotSetPositionOnce(15));
    // m_Controller.operator.start().onTrue(new ToggleVisionLEDs());
    // m_Controller.operator.back().onTrue(new SwitchToNextVisionPipeline());

    // controller.operator.rightBumper().and(controller.operator.leftBumper().negate()).onTrue(robotState.setColorWantedState(3));
    // controller.operator.rightBumper().and(controller.operator.leftBumper().negate()).whileTrue(new IntakeGroundCube());
    // controller.operator.rightBumper().and(controller.operator.leftBumper().negate()).onFalse(intakeCube.retractSpinNot());
    // m_Controller.operator.rightBumper().and(m_Controller.operator.leftBumper().negate()).onFalse(m_Superstructure.prepareCarry());

  }
  /*
    public void setThrustCoast() {
      drivetrain.setCoastMode();
    }

    public void setThrustBrake() {
      drivetrain.setBrakeMode();
    }
  */

  public void clearCanFaults() {
    pdh.clearStickyFaults();
  }

  public void printCanFaults() {
    pdh.getStickyFaults();
  }

  public void checkAutonomousSelection(Boolean force) {
    if (autonChooser.get() != null
        && (!choosenAutonomousCommand.equals(autonChooser.get()) || force)) {
      Long start = System.nanoTime();
      choosenAutonomousCommand = autonChooser.get();
      try {
        // TODO: re implement this
        choosenAutonomousCommand = autonChooser.get();
      } catch (Exception e) {
        System.out.println("[ERROR] could not find" + autonChooser.get().getName());
        System.out.println(e.toString());
      }

      Long totalTime = System.nanoTime() - start;
      System.out.println(
          "Autonomous Selected: ["
              + autonChooser.get().getName()
              + "] generated in "
              + (totalTime / 1000000.0)
              + "ms");
    }
  }

  public void checkAutonomousSelection() {
    checkAutonomousSelection(false);
  }

  public void checkAllaince() {
    // TODO: check the optional return instead of just .get()
    if (DriverStation.getAlliance().orElse(Alliance.Blue) != currentAlliance) {
      currentAlliance = DriverStation.getAlliance().orElse(Alliance.Blue);
      // Updated any things that need to change
      System.out.println("changed alliance");
      checkAutonomousSelection(true);
    }
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return choosenAutonomousCommand;
    // return drivetrain.swerveDriveAuto(1,0,0);
    // if(choosenAutonomousCommand == null) {
    //   //this is if for some reason checkAutonomousSelection is never called
    //   String chosen = autonChooser.get();
    //   chosen = "wall_3nSteal_3";
    //   PathPlannerAuto ppSwerveAuto = new PathPlannerAuto(chosen);
    //   return ppSwerveAuto.getauto();
    // }
    // choosenAutonomousCommand = new PathPlannerAuto("wall_3nSteal_3").getauto();
    // return choosenAutonomousCommand;
  }
}
