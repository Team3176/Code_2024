// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team3176.robot.subsystems.leds;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class LEDDefault extends CommandBase {
  // Import subsystems
  LEDSubsystem m_led;
  double startTime;

  public LEDDefault(LEDSubsystem led) {
    m_led = led;
    addRequirements(m_led);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
