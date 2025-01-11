// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Elevator;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public interface ElevatorIO {
  /** Creates a new ElevatorIO. */

  @AutoLog
  public static class ElevatorIOInputs {
      // insert inputs
  }

  public default void updateInputs(ElevatorIOInputs inputs) {}

  // insert methods from subsystem layer
}
