// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.AlgaeTilt;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AlgaeTilt extends SubsystemBase {
  private final AlgaeTiltIO io;
  private final AlgaeTiltIOInputsAutoLogged inputs = new AlgaeTiltIOInputsAutoLogged();

  /** Creates a new AlgaeIntake. */
  public AlgaeTilt(AlgaeTiltIO io) {
    this.io = io;
  }

  public void setDutyCycle(double dutyCycle) {
    io.setDutyCycle(dutyCycle);
  }
  
  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Algae Intake Arm", inputs);
  }
}
