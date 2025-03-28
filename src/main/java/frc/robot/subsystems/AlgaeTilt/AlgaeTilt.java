// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.AlgaeTilt;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
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

  public void setEncoder(double value) {
    io.setEncoder(value);
  }

  public void setPosition(double position) {
    io.setPosition(position);
  }
  
  public double getPositionRelative() {
    return inputs.armPositionRelative;
  }

  public double getPositionAbsolute() {
    return inputs.armPositionAbsolute;
  }

  public void stop() {
    io.setDutyCycle(0.0);
  }

  public Command setPositionCmd(double position) {
    return this.runEnd(
      () -> io.setPosition(position),
      () -> io.setPosition(position)
      );
  }


  public Command setDutyCycleCmd(DoubleSupplier duty) {
    return this.runEnd(
      () -> io.setDutyCycle(duty.getAsDouble()),
      () -> io.setDutyCycle(0.0));
  }
  
  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Algae Tilt", inputs);
  }
}
