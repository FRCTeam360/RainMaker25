// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Climber;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {
  private final ClimberIO io;
  private final ClimberIOInputsAutoLogged inputs = new ClimberIOInputsAutoLogged();
  
  /** Creates a new Climber. */
  public Climber(ClimberIO io) {
    this.io = io;
  }

  public void setWinchDutyCycle(double dutyCycle) {
    io.setWinchDutyCycle(dutyCycle);
  }

  public void setWheelDutyCycle(double dutyCycle) {
    io.setWheelDutyCycle(dutyCycle);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Climber", inputs);
  }
}
