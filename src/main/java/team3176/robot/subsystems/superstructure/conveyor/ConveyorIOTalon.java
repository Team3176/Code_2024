// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team3176.robot.subsystems.superstructure.conveyor;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.util.Units;
import team3176.robot.constants.Hardwaremap;

public class ConveyorIOTalon implements ConveyorIO {

  private TalonFX transferController =
      new TalonFX(Hardwaremap.shooterTransfer_CID, Hardwaremap.shooterTransfer_CBN);

  private TalonFXConfiguration configsWheelLower2 = new TalonFXConfiguration();

  public ConveyorIOTalon() {

    transferController.getConfigurator().apply(configsWheelLower2);
    // }
  }

  @Override
  public void updateInputs(ConveyorIOInputs inputs) {
    inputs.WheelVelocity =
        Units.rotationsToRadians(transferController.getVelocity().getValue());
    inputs.AppliedVolts = transferController.getMotorVoltage().getValue();
  }

  @Override
  public void setController(double velocity) {
    transferController.set(velocity);
  }
}
