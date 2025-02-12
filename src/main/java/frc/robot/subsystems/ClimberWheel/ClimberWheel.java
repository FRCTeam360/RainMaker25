// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.ClimberWheel;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimberWheel extends SubsystemBase {
  private final ClimberWheelIO io;
  private final ClimberWheelIOInputsAutoLogged inputs = new ClimberWheelIOInputsAutoLogged();

  /** Creates a new ClimberWheel. */
  public ClimberWheel(ClimberWheelIO io) {
    this.io = io;
  }

  public void setDutyCycle(double dutyCycle) {
    io.setDutyCycle(dutyCycle);
  }

  public Command setDutyCycleCmd(double dutyCycle) {
    return this.runEnd(() -> this.setDutyCycle(dutyCycle), () -> this.setDutyCycle(0));
  }

  public void setPosition(double position) {
    io.setPosition(position);
  }

  public Command setPositionCmd(double position) {
    return this.runEnd(() -> this.setPosition(position), () -> this.setPosition(position));
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Climber Wheel", inputs);
  }
}
