// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.ClimberWheel;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public interface ClimberWheelIO {
  /** Creates a new ClimberWheelIO. */
  @AutoLog
  public static class ClimberWheelIOInputs {
    public double wheelPosition = 0.0;
    public double wheelVelocity = 0.0;
    public double wheelDutyCycle = 0.0;
    public double wheelCurrent = 0.0;
    public double wheelTemp = 0.0;
  }

  public default void updateInputs(ClimberWheelIOInputs inputs) {}

  public void setDutyCycle(double dutyCycle);
}
