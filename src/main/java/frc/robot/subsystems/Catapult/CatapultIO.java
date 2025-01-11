// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Catapult;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public interface CatapultIO {
  /** Creates a new CatapultIO. */

  @AutoLog
  public static class CatapultIOInputs {
    // insert inputs
  }

  public default void updateInputs(CatapultIOInputs inputs) {}

  // insert subsystem methods
}
