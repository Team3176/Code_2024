// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team3176.robot.subsystems.controller;

import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class Controller {
  private static Controller instance;
  private static final Double DEADBAND = 0.02;

  public static Controller getInstance() {
    if (instance == null) {
      instance = new Controller();
    }
    return instance;
  }

  /* The Three Physical Controllers that we have */

  public final CommandJoystick transStick;
  public final CommandJoystick rotStick;
  public final CommandXboxController operator;
  public final CommandXboxController driver;
  public final CommandJoystick switchBox;

  /* First Part of Creating the Buttons on the Joysticks */

  public Controller() {
    /* Finish Creating the Objects */

    transStick = new CommandJoystick(1);
    rotStick = new CommandJoystick(0);
    operator = new CommandXboxController(2);
    switchBox = new CommandJoystick(3);
    driver = new CommandXboxController(4);
  }

  /**
   * @return The scales magnitude vector of the Y axis of TransStick
   */
  public double getForward() {
    if (Math.abs(driver.getLeftY()) < DEADBAND) {
      return -transStick.getY();
    }
    return -driver.getLeftY();
  }

  /**
   * @return The scales magnitude vector of the X axis of TransStick
   */
  public double getStrafe() {
    if (Math.abs(driver.getLeftX()) < DEADBAND) {
      return -transStick.getX();
    }
    return -driver.getLeftX();
  }

  /**
   * Scale is the power of 1 Deadband of 0.06
   *
   * @return The scales magnitude vector of the X axis of RotStick
   */
  public double getSpin() {

    if (Math.abs(driver.getRightX()) < DEADBAND) {
      return -rotStick.getX();
    }
    return -driver.getRightX();
  }

  public double getXboxJoyLeft() {
    // System.out.println("getLeftY = " + operator.getLeftY());
    return (-1 * operator.getLeftY());
  }

  public double getXboxJoyRight() {
    return (-1 * operator.getRightY());
  }
}
