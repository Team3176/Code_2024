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
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import team3176.robot.commands.WheelRadiusCharacterization;
import team3176.robot.commands.WheelRadiusCharacterization.Direction;
import team3176.robot.constants.Hardwaremap;
import team3176.robot.subsystems.Visualization;
import team3176.robot.subsystems.controller.Controller;
import team3176.robot.subsystems.drivetrain.Drivetrain;
import team3176.robot.subsystems.drivetrain.Drivetrain.orientationGoal;
import team3176.robot.subsystems.leds.LEDS;
import team3176.robot.subsystems.leds.LEDSubsystem;
import team3176.robot.subsystems.superstructure.*;
import team3176.robot.subsystems.superstructure.climb.Climb;
import team3176.robot.subsystems.superstructure.conveyor.Conveyor;
import team3176.robot.subsystems.superstructure.intake.Intake;
import team3176.robot.subsystems.superstructure.shooter.Shooter;
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
  private Trigger endMatchAlert = new Trigger(() -> DriverStation.getMatchTime() < 20);
  private Trigger hasNote = new Trigger(() -> Conveyor.getInstance().hasNote());
  private Trigger shooterOverride;
  private Trigger ampOverride;
  private Trigger intakeOverride;
  private Trigger visionOverride;
  private LEDS ledsRio;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    controller = Controller.getInstance();
    superstructure = Superstructure.getInstance();
    drivetrain = Drivetrain.getInstance();

    leds = LEDSubsystem.getInstance();
    ledsRio = LEDS.getInstance();
    endMatchAlert.onTrue(leds.EndgameStart());

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
    leds.setDefaultCommand(leds.DefaultLED());
    // These all need to be sped up
    NamedCommands.registerCommand("shoot", new WaitCommand(1.0));
    // NamedCommands.registerCommand(
    //     "shoot",
    //     superstructure
    //         .aimClose()
    //         .alongWith(new WaitCommand(0.5).andThen(superstructure.shoot().withTimeout(0.3)))
    //         .withTimeout(0.8)
    //         .withName("shooting"));
    NamedCommands.registerCommand(
        "shootAim",
        superstructure
            .aimClose()
            .asProxy()
            .raceWith(drivetrain.driveAndAim(() -> 0, () -> 0))
            .raceWith(
                new WaitCommand(0.5)
                    .andThen(superstructure.shoot().withTimeout(0.5).asProxy())
                    .withName("shooting")));
    NamedCommands.registerCommand(
        "shootLookup",
        superstructure
            .aimShooterLookup()
            .raceWith(drivetrain.driveAndAim(() -> 0, () -> 0))
            .raceWith(
                new WaitCommand(0.7) // .until(() -> superstructure.readyToShoot())
                    .andThen(superstructure.shoot().withTimeout(0.4))
                    .withName("shooting")));
    NamedCommands.registerCommand(
        "shootLookupQuick",
        superstructure
            .aimShooterLookup()
            .raceWith(drivetrain.driveAndAim(() -> 0, () -> 0))
            .raceWith(
                new WaitCommand(0.1) // .until(() -> superstructure.readyToShoot())
                    .andThen(superstructure.shoot().withTimeout(0.4))
                    .withName("shooting")));
    NamedCommands.registerCommand(
        "chaseNote", drivetrain.chaseNote().raceWith(superstructure.intakeNote()).withTimeout(1.5));

    Command chaseNoteAuto =
        drivetrain
            .autoChaseTarget(orientationGoal.NOTECAM)
            .until(() -> Conveyor.getInstance().hasNote())
            .withTimeout(2.0);
    NamedCommands.registerCommand("deployIntake", Intake.getInstance().deployPivot());
    // using schedule to prevent intake from being cancelled if the path ends
    NamedCommands.registerCommand("intake", superstructure.intakeNote().withTimeout(3.0));
    NamedCommands.registerCommand(
        "aimSpeaker",
        drivetrain
            .autoChaseTarget(orientationGoal.SPEAKER)
            .alongWith(superstructure.aimShooterLookup())
            .withTimeout(1.0));
    NamedCommands.registerCommand("score", superstructure.shoot().withTimeout(0.5));

    autonChooser = new LoggedDashboardChooser<>("autonChoice", AutoBuilder.buildAutoChooser());

    SmartDashboard.putData("Auton Choice", autonChooser.getSendableChooser());
    configureBindings();
  }

  private void configureBindings() {
    /*
     * overrides
     */
    shooterOverride = controller.switchBox.button(1);
    ampOverride = controller.switchBox.button(2);
    intakeOverride = controller.switchBox.button(3);
    visionOverride = controller.switchBox.button(4);
    /*
     * Translation Stick
     */
    /*     controller
    .transStick
    .button(1)
    .whileTrue(new WheelRadiusCharacterization(drivetrain, Direction.CLOCKWISE)); */
    controller
        .transStick
        .button(1)
        .whileTrue(
            drivetrain
                .swerveDriveJoysticks(
                    () -> controller.getForward(),
                    () -> controller.getStrafe(),
                    () -> controller.getSpin() * 1.5)
                .withName("boost drive"));
    controller
        .transStick
        .button(2)
        .onTrue(
            superstructure
                .intakeNote()
                .withName("intakeNote")
                .alongWith(ledsRio.Intaking().asProxy()))
        .onFalse(superstructure.retractIntakePivot());

    controller
        .transStick
        .button(3)
        .whileTrue(
            drivetrain
                .chaseNote()
                .until(() -> Conveyor.getInstance().isLaserIntakeSide())
                .alongWith(superstructure.intakeNote())
                .alongWith(ledsRio.AutoDrive().asProxy())
                .withName("intakeAutoDrive"));
    /*
        controller
        .transStick
        .button(4)
        .whileTrue(
            drivetrain
                .chaseNoteTeleo(
                    () -> controller.getForward(),
                    () -> controller.getStrafe(),
                    () -> controller.getSpin())
                .alongWith(superstructure.intakeNote()));
    */
    controller.transStick.button(5).onTrue(drivetrain.resetPoseToVisionCommand());
    controller
        .transStick
        .button(10)
        .whileTrue(drivetrain.swerveDefenseCommand().withName("swerveDefense"));

    /*
     *  Rotation Stick
     */
    controller.rotStick.button(1).whileTrue(superstructure.shoot().withName("shoot"));
    controller
        .rotStick
        .button(2)
        .whileTrue(
            Commands.either(
                superstructure.aimShooterTune().withName("shooterAimOverride"),
                Commands.either(
                    drivetrain
                        .driveAndAimPass(
                            () -> controller.getForward(), () -> controller.getStrafe())
                        .asProxy()
                        .alongWith(superstructure.aimPass())
                        .withName("aimTuneAndDrive"),
                    drivetrain
                        .driveAndAim(() -> controller.getForward(), () -> controller.getStrafe())
                        .asProxy()
                        .alongWith(superstructure.aimShooterLookup())
                        .withName("aimTuneAndDrive"),
                    () -> Shooter.getInstance().getDistance() > 9.0),
                shooterOverride));
    controller
        .rotStick
        .button(3)
        .whileTrue(superstructure.aimShooterTune().withName("aimShooterTune"));
    // this is reverse switch once we prove out the auto score
    controller
        .rotStick
        .button(4)
        .whileTrue(
            Commands.either(
                superstructure
                    .aimAmp(true)
                    .withName("aimAmp")
                    .alongWith(ledsRio.Amping().asProxy()),
                superstructure
                    .aimAmp(false)
                    .withName("aimAmpDrive")
                    .alongWith(ledsRio.Amping().asProxy()),
                ampOverride))
        .onFalse(Climb.getInstance().stow());
    controller
        .rotStick
        .button(8)
        .whileTrue(new InstantCommand(drivetrain::resetFieldOrientation, drivetrain));

    /*
     * Operator
     */
    controller.operator.x().whileTrue(superstructure.spit());
    controller.operator.b().whileTrue(Intake.getInstance().spinIntakeRollersSlow());
    controller
        .operator
        .leftBumper()
        .whileTrue(
            superstructure
                .moveClimbLeftRightPosition(
                    () -> -controller.operator.getLeftY(), () -> -controller.operator.getRightY())
                .alongWith(ledsRio.Climbing().asProxy()))
        .onFalse(superstructure.stopClimbLeftRight());
    controller.operator.leftBumper().onTrue(Intake.getInstance().climbIntake());
    controller.operator.leftTrigger().whileTrue(superstructure.aimClose());
    controller.operator.rightTrigger().whileTrue(superstructure.shoot());
    /*
    controller
        .operator
        .rightBumper()
        .whileTrue(superstructure.moveClimbRightPosition(() -> controller.operator.getRightY()))
        .onFalse(superstructure.stopClimbRight());
        */
    controller.operator.povUp().whileTrue(Intake.getInstance().EmergencyHold());
    controller.operator.povDown().whileTrue(Intake.getInstance().manualDown());
    // controller.operator.povDown().onTrue(superstructure.intakeNote());

    controller
        .operator
        .start()
        .and(controller.operator.povLeft())
        .onTrue(Intake.getInstance().EmergencyHold());

    controller
        .switchBox
        .button(5)
        .whileTrue(new WheelRadiusCharacterization(drivetrain, Direction.CLOCKWISE));

    controller
        .switchBox
        .button(4)
        .onTrue(drivetrain.setVisionOverride(true))
        .onFalse(drivetrain.setVisionOverride(false));
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
    return autonChooser.get();
  }
}
