// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.CoralOuttake;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CoralOuttake extends SubsystemBase {
  private final CoralOuttakeIO io;
  private final CoralOuttakeIOInputsAutoLogged inputs = new CoralOuttakeIOInputsAutoLogged();

  /** Creates a new CoralOutake. */
  public CoralOuttake(CoralOuttakeIO io) {
    this.io = io;
  }

  public void setDutyCycle(double speed) {
    io.setDutyCycle(speed);
  }

  public void getOuttakeSensor() {
    io.getOuttakeSensor();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    io.updateInputs(inputs);
    Logger.processInputs("Coral Outtake", inputs);
  }
}
