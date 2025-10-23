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
    public double algaeShooterFrontVoltage = 0.0;
    public double algaeShooterFrontVelocity = 0.0;
    public double algaeShooterFrontPosition = 0.0;
    public double algaeShooterFrontCurrent = 0.0;
    public double algaeShooterFrontTemperature = 0.0;

    public double algaeShooterBackVoltage = 0.0;
    public double algaeShooterBackVelocity = 0.0;
    public double algaeShooterBackPosition = 0.0;
    public double algaeShooterBackCurrent = 0.0;
    public double algaeShooterBackTemperature = 0.0;
    // insert inputs
  }

  public default void updateInputs(AlgaeShooterIOInputs inputs) {}

  public void setDutyCycle(double dutyCycle);

  public void setVelocity(double velocity);

  public void stop();
}
