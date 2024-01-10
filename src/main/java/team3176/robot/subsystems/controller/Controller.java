// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team3176.robot.subsystems.controller;

import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import team3176.robot.constants.ControllerConstants;

public class Controller {
  private static Controller instance;

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

  /* First Part of Creating the Buttons on the Joysticks */

  public Controller() {
    /* Finish Creating the Objects */

    transStick = new CommandJoystick(ControllerConstants.TRANS_ID);
    rotStick = new CommandJoystick(ControllerConstants.ROT_ID);
    operator = new CommandXboxController(ControllerConstants.OP_ID);
  }

  /**
   * Scale is the power of 1 Deadband of 0.06
   *
   * @return The scales magnitude vector of the Y axis of TransStick if it breaks deadband
   */
  public double getForward() {
    if (Math.abs(transStick.getY()) < 0.06) return 0.0;
    return ControllerConstants.FORWARD_AXIS_INVERSION * Math.pow(transStick.getY(), 1);
  }

  /**
   * Scale is the power of 1 Deadband of 0.06
   *
   * @return The scales magnitude vector of the X axis of TransStick if it breaks deadband
   */
  public double getStrafe() {
    if (Math.abs(transStick.getX()) < 0.06) return 0.0;
    return ControllerConstants.STRAFE_AXIS_INVERSION * Math.pow(transStick.getX(), 1);
  }

  /**
   * Scale is the power of 1 Deadband of 0.06
   *
   * @return The scales magnitude vector of the X axis of RotStick if it breaks deadband
   */
  public double getSpin() {
    if (Math.abs(rotStick.getX()) < 0.06) return 0.0;
    return ControllerConstants.SPIN_AXIS_INVERSION * rotStick.getX();
  }
}
