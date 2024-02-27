// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team3176.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import team3176.robot.commands.drivetrain.*;
import team3176.robot.constants.Hardwaremap;
import team3176.robot.subsystems.Visualization;
import team3176.robot.subsystems.controller.Controller;
import team3176.robot.subsystems.drivetrain.Drivetrain;
import team3176.robot.subsystems.leds.LEDSubsystem;
import team3176.robot.subsystems.superstructure.*;
import team3176.robot.subsystems.superstructure.intake.Intake;
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
  private Drivetrain drivetrain;
  private LEDSubsystem leds;
  private Superstructure superstructure;
  private PhotonVisionSystem vision;
  private Visualization visualization;
  private LoggedDashboardChooser<Command> autonChooser;
  private Command choosenAutonomousCommand = new WaitCommand(1.0);
  private Alliance currentAlliance = Alliance.Blue;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    controller = Controller.getInstance();
    superstructure = Superstructure.getInstance();
    drivetrain = Drivetrain.getInstance();

    leds = new LEDSubsystem();

    // superstructure = Superstructure.getInstance();
    visualization = new Visualization();
    if (Constants.VISION_CONNECTED) {
      vision = PhotonVisionSystem.getInstance();
    }

    pdh = new PowerDistribution(Hardwaremap.PDH_CID, ModuleType.kRev);

    drivetrain.setDefaultCommand(
        drivetrain
            .swerveDriveJoysticks(
                () -> controller.getForward(),
                () -> controller.getStrafe(),
                () -> controller.getSpin())
            .withName("default drive"));
    NamedCommands.registerCommand(
        "shoot", new WaitCommand(0.5).alongWith(new PrintCommand("shoot")).withName("shooting"));

    autonChooser = new LoggedDashboardChooser<>("autonChoice", AutoBuilder.buildAutoChooser());

    SmartDashboard.putData("Auton Choice", autonChooser.getSendableChooser());
    configureBindings();
  }

  private void configureBindings() {
    controller.transStick.button(5).onTrue(drivetrain.resetPoseToVisionCommand());
    controller
        .transStick
        .button(10)
        .whileTrue(
            new InstantCommand(drivetrain::setBrakeMode)
                .andThen(drivetrain.swerveDefenseCommand())
                .withName("swerveDefense"));
    /* controller
           .rotStick
           .button(2)
           .whileTrue(
               drivetrain
                   .driveAndAim(() -> controller.getForward(), () -> controller.getStrafe())
                   .alongWith(Shooter.getInstance().aim()));
    */
    controller
        .rotStick
        .button(8)
        .whileTrue(new InstantCommand(drivetrain::resetFieldOrientation, drivetrain));

    controller
        .operator
        .b()
        .whileTrue(
            superstructure
                .shooterPivotPID(90)
                .alongWith(new PrintCommand("shooter"))
                .withName("shooter_pivot"))
        .onFalse(superstructure.shooterPivotPID(0));

    /*     controller
    .rotStick
    .button(1)
    .whileTrue(Shooter.getInstance().aim()); */

    controller.rotStick.button(1).whileTrue(superstructure.shoot());
    controller.rotStick.button(2).whileTrue(superstructure.aimShooter());
    controller
        .rotStick
        .button(3)
        .whileTrue(Intake.getInstance().spinIntakeUntilPivot())
        .onFalse(Intake.getInstance().stopRollers());
    controller.rotStick.button(4).onTrue(Intake.getInstance().intakeNote());
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
    return choosenAutonomousCommand;
  }
}
