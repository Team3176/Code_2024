// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team3176.robot.subsystems.superstructure.transfer;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.util.Units;
import team3176.robot.constants.Hardwaremap;

public class TransferIOTalon implements TransferIO {

  private TalonFX transferController =
      new TalonFX(Hardwaremap.shooterTransfer_CID, Hardwaremap.shooterWheelLower_CBN2);

  private TalonFXConfiguration configsWheelLower2 = new TalonFXConfiguration();

  public TransferIOTalon() {

    transferController.getConfigurator().apply(configsWheelLower2);
    // }
  }

  @Override
  public void updateInputs(TransferIOInputs inputs) {
    inputs.transferWheelVelocity =
        Units.rotationsToRadians(transferController.getVelocity().getValue());
    inputs.transferAppliedVolts = transferController.getMotorVoltage().getValue();
  }

  @Override
  public void setTransferController(double velocity) {
    transferController.set(velocity);
  }
}
