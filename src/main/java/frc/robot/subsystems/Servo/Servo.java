// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Servo;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Servo extends SubsystemBase {
    private final ServoIO io;
    private final ServoIOInputsAutoLogged inputs = new ServoIOInputsAutoLogged();
  /** Creates a new Servo. */
  public Servo(ServoIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Servo", inputs);
    // This method will be called once per scheduler run
  }
}
