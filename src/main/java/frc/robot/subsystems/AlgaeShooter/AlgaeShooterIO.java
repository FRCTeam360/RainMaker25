// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.AlgaeShooter;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public interface AlgaeShooterIO {
  /** Creates a new AlgaeShooterIO. */
  @AutoLog
  public static class AlgaeShooterIOInputs {
    public double algaeShooterVoltage = 0.0;
    public double algaeShooterVelocity = 0.0;
    public double algaeShooterPosition = 0.0;
    public double algaeShooterStatorCurrent = 0.0;
    public double algaeShooterSupplyCurrent = 0.0;
    public double algaeShooterTemperature = 0.0;
    // insert inputs
  }

  public default void updateInputs(AlgaeShooterIOInputs inputs) {}

  public void setDutyCycle(double dutyCycle);

  public void setVelocity(double velocity);

  public void setPosition(double position);
}
