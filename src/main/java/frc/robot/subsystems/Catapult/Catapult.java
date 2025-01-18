// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Catapult;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Catapult extends SubsystemBase {
  private final CatapultIO io;
  private final CatapultIOInputsAutoLogged inputs = new CatapultIOInputsAutoLogged();

  /** Creates a new Catapult. */
  public Catapult(CatapultIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    io.updateInputs(inputs);
    Logger.processInputs("Catapult", inputs);
  }
}
