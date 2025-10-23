// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Elevator;

import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO {
  /** Creates a new ElevatorIO. */
  @AutoLog
  public static class ElevatorIOInputs {
    public double elevatorStatorCurrent = 0.0;
    public double elevatorSupplyCurrent = 0.0;
    public double elevatorVoltage = 0.0;
    public double elevatorVelocity = 0.0;
    public double elevatorPosition = 0.0;
    public boolean elevatorSensor = false;
    // insert inputs
  }

  public default void updateInputs(ElevatorIOInputs inputs) {}

  public void setElevatorPostion(double height);

  public void setDutyCycle(double dutyCycle);

  public void stop();

  public void setEncoder(double value);
}
