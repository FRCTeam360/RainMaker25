// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.CoralIntake;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public interface CoralIntakeIO {
  /** Creates a new CoralIntakeIO. */

  @AutoLog
  public static class CoralIntakeIOInputs {
    // insert inputs
  }

  public default void updateInputs(CoralIntakeIOInputs inputs) {}

  // insert subsystem methods
}