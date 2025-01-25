// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.AlgaeArm;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.CoralIntake.CoralIntakeIOInputsAutoLogged;

public class AlgaeArm extends SubsystemBase {
  private final AlgaeArmIO io;
  private final AlgaeArmIOInputsAutoLogged inputs = new AlgaeArmIOInputsAutoLogged();

  /** Creates a new AlgeaArm. */
  public AlgaeArm(AlgaeArmIO io) {
    this.io = io;
  }

  public void setPosition(double position){
    io.setPosition(position);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    io.updateInputs(inputs);
    Logger.processInputs("Algea Arm", inputs);
  }
}
