// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team3176.robot;

import java.util.HashMap;
import java.util.Map;
import java.util.function.BiConsumer;

import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.inputs.LoggedPowerDistribution;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import team3176.robot.Constants.RobotType;
import team3176.robot.subsystems.drivetrain.Drivetrain;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends LoggedRobot{
  private Command autonomousCommand;

  private RobotContainer robotContainer;
  Thread fisheyeThread;
  private static final boolean FISHEYE_CAMERA = false;
  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    // TODO: Char restore this fxnality  -- mod build.gradle: see prepForState 723516
    System.out.println("[Init] Starting AdvantageKit");
    Logger.recordMetadata("Robot", Constants.getRobot().toString());
    Logger.recordMetadata("RuntimeType", getRuntimeType().toString());
    Logger.recordMetadata("ProjectName", BuildConstants.MAVEN_NAME);
    Logger.recordMetadata("BuildDate", BuildConstants.BUILD_DATE);
    Logger.recordMetadata("GitSHA", BuildConstants.GIT_SHA);
    Logger.recordMetadata("GitDate", BuildConstants.GIT_DATE);
    Logger.recordMetadata("GitBranch", BuildConstants.GIT_BRANCH);
    final String GitDirty = "GitDirty";
    switch (BuildConstants.DIRTY) {
      case 0:
        Logger.recordMetadata(GitDirty, "All changes committed");
        break;
      case 1:
        Logger.recordMetadata(GitDirty, "Uncomitted changes");
        break;
      default:
        Logger.recordMetadata(GitDirty, "Unknown");
        break;
    }
    switch (Constants.getMode()) {
      case REAL:
        String folder = Constants.logFolders.get(Constants.getRobot());
        if (folder != null) {
          try{
            Logger.addDataReceiver(new WPILOGWriter("/home/lvuser/logs/"));
          }
          catch(Error e) {
            System.out.println("[Error] failed to start local log file");
          }
        }
        Logger.addDataReceiver(new NT4Publisher());
        if (Constants.getRobot() == RobotType.ROBOT_2023C) {
          LoggedPowerDistribution.getInstance(50, ModuleType.kRev);
        }
        break;

      case SIM:
        System.out.println("[init] starting simulation");
        Logger.addDataReceiver(new WPILOGWriter("./log/"));
        Logger.addDataReceiver(new NT4Publisher());
        break;

      case REPLAY:
        String path = LogFileUtil.findReplayLog();
        Logger.setReplaySource(new WPILOGReader(path));
        Logger.addDataReceiver(new NT4Publisher());
        Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(path, "_sim")));
        break;
    }
    setUseTiming(Constants.getMode() != Constants.Mode.REPLAY);
    Logger.start();


    robotContainer = new RobotContainer();
    SmartDashboard.putData(CommandScheduler.getInstance());
    
    if(FISHEYE_CAMERA)
    {
      fisheyeThread = new Thread( () -> {
        UsbCamera fisheye = CameraServer.startAutomaticCapture();
        fisheye.setResolution(640,480);
        //not using this at the return so commenting for linting: CvSource outputStream = 
        CameraServer.putVideo("fisheye", 640, 480);
      });
      fisheyeThread.setDaemon(true);
      fisheyeThread.start();
    }
    
    Map<String, Integer> commandCounts = new HashMap<>();
    BiConsumer<Command, Boolean> logCommandFunction =
        (Command command, Boolean active) -> {
          String name = command.getName();
          int count = commandCounts.getOrDefault(name, 0) + (active ? 1 : -1);
          commandCounts.put(name, count);
          Logger.recordOutput(
                  "CommandsUnique/" + name + "_" + Integer.toHexString(command.hashCode()), active);
          Logger.recordOutput("CommandsAll/" + name, count > 0);
        };
    CommandScheduler.getInstance()
        .onCommandInitialize(
            (Command command) -> {
              logCommandFunction.accept(command, true);
            });
    CommandScheduler.getInstance()
        .onCommandFinish(
            (Command command) -> {
              logCommandFunction.accept(command, false);
            });
    CommandScheduler.getInstance()
        .onCommandInterrupt(
            (Command command) -> {
              logCommandFunction.accept(command, false);
            });

  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {
    robotContainer.setArmCoast();
  }

  @Override
  public void disabledPeriodic() {
    robotContainer.checkAutonomousSelection();
    robotContainer.checkAllaince();
  }

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    robotContainer.clearCanFaults();
    robotContainer.setArmBrake();
    robotContainer.setThrustBrake();
    autonomousCommand = robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (autonomousCommand != null) {
      autonomousCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    //scheduled command executes from init()
  }

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    robotContainer.clearCanFaults();
        robotContainer.setArmBrake();
    robotContainer.setThrustCoast();
    if (autonomousCommand != null) {
      autonomousCommand.cancel();
    }
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    //command scheduler is responsible for actions
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
    //nan
  }

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {
    //nan
  }

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {
    //nan
  }
}
