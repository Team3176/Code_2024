// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

package team3176.robot.constants;

import java.util.Map;

import edu.wpi.first.wpilibj.RobotBase;
import team3176.robot.util.God.Alert;
import team3176.robot.util.God.Alert.AlertType;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class RobotConstants {
  private static final RobotType robot = RobotType.PROTCHASSIS;
  public static final double loopPeriodSecs = 0.02;
  public static final boolean tuningMode = false;

  private static final Alert invalidRobotAlert =
      new Alert("Invalid robot selected, using competition robot as default.",
          AlertType.ERROR);

  public static RobotType getRobot() {
    if (RobotBase.isReal()) {
      if (robot == RobotType.SIMBOT) { // Invalid robot selected
        invalidRobotAlert.set(true);
        return RobotType.PRODBOT;
      } else {
        return robot;
      }
    } else {
      return robot;
    }
  }

  public static Mode getMode() {
    switch (getRobot()) {
      case EBOT:
        return RobotBase.isReal() ? Mode.REAL : Mode.REPLAY;

      case PRODBOT:
        return RobotBase.isReal() ? Mode.REAL : Mode.REPLAY;
      
        case PROTCHASSIS:
        return RobotBase.isReal() ? Mode.REAL : Mode.REPLAY;

      case SIMBOT:
        return Mode.SIM;

      default:
        return Mode.REAL;
    }
  }

  public static final Map<RobotType, String> logFolders =
      Map.of(RobotType.PRODBOT, "/media/sda1"
      
      
      
      );

  public static boolean isEBOT() {
    if (getRobot() == RobotType.EBOT) {
      return true;
    } else return false;
  } 
  
  public static boolean isPRODBOT() {
    if (getRobot() == RobotType.PRODBOT) {
      return true;
    } else return false;
  } 

  public static boolean isPROTCHASSIS() {
    if (getRobot() == RobotType.PRODBOT) {
      return true;
    } else return false;
  } 

  public static boolean isSIMBOT() {
    if (getRobot() == RobotType.PRODBOT) {
      return true;
    } else return false;
  } 
  
  public static enum RobotType {
    EBOT, PRODBOT, PROTCHASSIS, SIMBOT
  }


  public static enum Mode {
    REAL, REPLAY, SIM
  }

  public static enum Status {
    STABLE, OK, OPTIONALCHECK, WARNING, GOOD, ERROR, CONE, CUBE, CONEFLASH, CUBEFLASH, NONE
  }
}