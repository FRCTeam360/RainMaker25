// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.ClimberWinch;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public interface ClimberWinchIO {
  /** Creates a new ClimberIO. */

  @AutoLog
  public static class ClimberWinchIOInputs {
    public double winchVelocity = 0.0;
    public double winchPosition = 0.0;
    public double winchDutyCycle = 0.0;
  }

  public default void updateInputs(ClimberWinchIOInputs inputs) {}

  public void setDutyCycle(double dutyCycle);
}
