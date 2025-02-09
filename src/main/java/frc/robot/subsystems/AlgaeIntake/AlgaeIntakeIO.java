// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.AlgaeIntake;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public interface AlgaeIntakeIO {
  /** Creates a new AlgaeIntakeIO. */
  public static class AlgaeIntakeIOInputs {
    private double intakePosition = 0.0;
    private double intakeDutyCycle = 0.0;
    private double intakeVelocity = 0.0;
  }

  public default void updateInputs(AlgaeIntakeIOInputs inputs) {}
}
