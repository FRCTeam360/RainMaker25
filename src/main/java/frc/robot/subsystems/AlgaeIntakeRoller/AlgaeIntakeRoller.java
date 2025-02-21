// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.AlgaeIntakeRoller;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AlgaeIntakeRoller extends SubsystemBase {
  private final AlgaeIntakeRollerIO io;
  private final AlgaeIntakeRollerIOInputsAutoLogged inputs = new AlgaeIntakeRollerIOInputsAutoLogged();

  /** Creates a new AlgaeIntakeRoller. */
  public AlgaeIntakeRoller(AlgaeIntakeRollerIO io) {
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
