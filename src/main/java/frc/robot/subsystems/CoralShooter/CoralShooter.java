// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.CoralShooter;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CoralShooter extends SubsystemBase {
  private final CoralShooterIO io;
  public final CoralShooterIOInputsAutoLogged inputs = new CoralShooterIOInputsAutoLogged();

  /** Creates a new CoralOutake. */
  public CoralShooter(CoralShooterIO io) {
    this.io = io;
  }

  public void setDutyCycle(double speed) {
    io.setDutyCycle(speed);
  }

  public void stop() {
    io.stop();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    io.updateInputs(inputs);
    Logger.processInputs("Coral Outtake", inputs);
  }
}
