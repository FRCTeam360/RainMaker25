// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.CoralOuttake;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public interface CoralOuttakeIO {
  /** Creates a new CoralOutakeIO. */
  
  @AutoLog
  public static class CoralOuttakeIOInputs {
    // insert inputs
  }

  public default void updateInputs(CoralOuttakeIOInputs inputs) {}
  
  // insert subsystem methods
}
