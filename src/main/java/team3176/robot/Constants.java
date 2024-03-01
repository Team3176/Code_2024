// Copyright (c) 2023 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package team3176.robot;

import edu.wpi.first.wpilibj.RobotBase;
import java.util.Map;

public final class Constants {
  private static final RobotType robot = RobotType.ROBOT_SIMBOT;
  public static final double LOOP_PERIODIC_SECS = 0.02;
  public static final boolean TUNING_MODE = true;
  public static final boolean VISION_CONNECTED = true;
  public static boolean invalidRobotAlertSent = false;

  public static RobotType getRobot() {
    if (!isHALdisable && RobotBase.isReal()) {
      if (robot == RobotType.ROBOT_SIMBOT) { // Invalid robot selected
        if (!invalidRobotAlertSent) {
          invalidRobotAlertSent = true;
        }
        return RobotType.ROBOT_2024C;
      } else {
        return robot;
      }
    } else {
      return robot;
    }
  }

  public static Mode getMode() {
    switch (getRobot()) {
      case ROBOT_2024C:
      case CTRL_BOARD:
        return RobotBase.isReal() ? Mode.REAL : Mode.REPLAY;

      case ROBOT_SIMBOT:
        return Mode.SIM;

      default:
        return Mode.REAL;
    }
  }

  public static final Map<RobotType, String> logFolders =
      Map.of(RobotType.ROBOT_2024C, "/media/sda1/");

  public static enum RobotType {
    ROBOT_2024C,
    CTRL_BOARD,
    ROBOT_SIMBOT
  }

  public static enum Mode {
    REAL,
    REPLAY,
    SIM
  }

  public static enum Status {
    STABLE,
    OK,
    OPTIONALCHECK,
    WARNING,
    GOOD,
    ERROR,
    CONE,
    CUBE,
    CONEFLASH,
    CUBEFLASH,
    NONE
  }

  // Function to disable HAL interaction when running without native libs
  public static boolean isHALdisable = false;

  public static void disableHAL() {
    isHALdisable = true;
  }

  /** Checks whether the robot the correct robot is selected when deploying. */
  public static void main(String... args) {
    if (robot == RobotType.ROBOT_SIMBOT) {
      System.err.println("Cannot deploy, invalid robot selected: " + robot.toString());
      System.exit(1);
    }
  }
}
