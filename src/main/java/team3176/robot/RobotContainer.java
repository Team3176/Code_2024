// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team3176.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import java.io.File;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import team3176.robot.Constants.Mode;
import team3176.robot.commands.drivetrain.*;
import team3176.robot.constants.Hardwaremap;
import team3176.robot.subsystems.RobotState;
import team3176.robot.subsystems.controller.Controller;
import team3176.robot.subsystems.drivetrain.Drivetrain;
import team3176.robot.subsystems.vision.PhotonVisionSystem;
import team3176.robot.subsystems.superstructure.*;

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
  private final Drivetrain drivetrain;
  private final RobotState robotState;
  private PhotonVisionSystem vision;
  private Intake intake;
  private LoggedDashboardChooser<Command> autonChooser;
  private Command choosenAutonomousCommand = new WaitCommand(1.0);
  private Alliance currentAlliance = Alliance.Blue;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    controller = Controller.getInstance();
    drivetrain = Drivetrain.getInstance();
    robotState = RobotState.getInstance();
    intake = Intake.getInstance();
    if(Constants.VISION_CONNECTED){
      vision = PhotonVisionSystem.getInstance();
    }
    
    pdh = new PowerDistribution(Hardwaremap.PDH_CID, ModuleType.kRev);
    drivetrain.setDefaultCommand(
        drivetrain.swerveDrivePercent(
            () -> controller.getForward() * 0.7,
            () -> controller.getStrafe() * 0.7,
            () -> controller.getSpin() * 3).withName("default drive"));
    if(Constants.getMode() == Mode.SIM) {
      drivetrain.setDefaultCommand(
        drivetrain.swerveDrivePercent(
            () -> controller.getForward() * 0.7,
            () -> controller.getStrafe() * 0.7,
            () -> controller.getSpin() * 3,false).withName("default drive"));
    }
    NamedCommands.registerCommand("shoot", new WaitCommand(0.5).alongWith(new PrintCommand("shoot")).withName("shooting"));
     NamedCommands.registerCommand("intake", new WaitCommand(0.5).alongWith(new PrintCommand("intake")).withName("intaking"));
    // autonChooser.addDefaultOption("wall_3_cube_poop_4_steal", "wall_3_cube_poop_4_steal");
    autonChooser = new LoggedDashboardChooser<>("autonChoice",  AutoBuilder.buildAutoChooser());
    // File paths = new File(Filesystem.getDeployDirectory(), "pathplanner");
    // for (File f : paths.listFiles()) {
    //   if (!f.isDirectory()) {
    //     String s = f.getName().split("\\.", 0)[0];
    //     autonChooser.addOption(s, s);
    //   }
    // }

    SmartDashboard.putData("Auton Choice", autonChooser.getSendableChooser());

    configureBindings();
  }

  private void configureBindings() {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`

    // m_Controller.getTransStick_Button1().onFalse(new InstantCommand(() ->
    // m_Drivetrain.setTurbo(false), m_Drivetrain));
    //controller.transStick.button(2).whileTrue(drivetrain.pathfind("shoot"));
    //controller.transStick.button(3).whileTrue(drivetrain.pathfind("pickup"));
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

    // m_Controller.getRotStick_Button2().whileTrue(new FlipField);
    // controller.transStick.button(14).and(controller.transStick.button(15)).onTrue(drivetrain.setFieldCentric());
    // controller.transStick.button(14).and(controller.transStick.button(16)).onTrue(drivetrain.setRobotCentric());

    // controller.rotStick.button(1).whileTrue(new CubeChase(
    controller
        .rotStick
        .button(1)
        .whileTrue(
            drivetrain.swerveDrivePercent(
                () -> controller.getForward() * -1.0,
                () -> controller.getStrafe() * 1.0,
                () -> controller.getSpin() * 7));

    controller
        .rotStick
        .button(2)
        .whileTrue(drivetrain.SpinLockDrive(controller::getForward, controller::getStrafe));

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
    
    controller.transStick.button(1).onTrue(intake.runIntake(-1));
    controller.transStick.button(1).onFalse(intake.runIntake(0.0));

    // m_Controller.operator.start().onTrue(new ToggleVisionLEDs());
    // m_Controller.operator.back().onTrue(new SwitchToNextVisionPipeline());

    // controller.operator.rightBumper().and(controller.operator.leftBumper().negate()).onTrue(robotState.setColorWantedState(3));
    // controller.operator.rightBumper().and(controller.operator.leftBumper().negate()).whileTrue(new IntakeGroundCube());
    // controller.operator.rightBumper().and(controller.operator.leftBumper().negate()).onFalse(intakeCube.retractSpinNot());
    // m_Controller.operator.rightBumper().and(m_Controller.operator.leftBumper().negate()).onFalse(m_Superstructure.prepareCarry());

  }

  public void setThrustCoast() {
    drivetrain.setCoastMode();
  }

  public void setThrustBrake() {
    drivetrain.setBrakeMode();
  }

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
        choosenAutonomousCommand =
            autonChooser.get();
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
    //return drivetrain.swerveDriveAuto(1,0,0);
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
