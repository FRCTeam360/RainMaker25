// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.AlgaeRoller;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AlgaeRoller extends SubsystemBase {
  private final AlgaeRollerIO io;
  private final AlgaeRollerIOInputsAutoLogged inputs = new AlgaeRollerIOInputsAutoLogged();

  /** Creates a new AlgaeIntakeRoller. */
  public AlgaeRoller(AlgaeRollerIO io) {
    this.io = io;
  }

  public void setDutyCycle(double dutyCycle) {
    io.setDutyCycle(dutyCycle);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Algae Intake Roller", inputs);
  }
}
