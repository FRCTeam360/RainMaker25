// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.AlgaeArm;

import org.littletonrobotics.junction.AutoLog;

public interface AlgaeArmIO {
  /** Creates a new AlgeaArmIO. */
  @AutoLog
  public static class AlgaeArmIOInputs {
    public double algaeArmCurrent = 0.0;
    public double algaeArmVoltage = 0.0;
    public double algaeArmVelocity = 0.0;
    public double algaeArmAngle = 0.0;
    public double algaeArmTemp = 0.0;
  }

  public default void updateInputs(AlgaeArmIOInputs inputs) {}

  public void setDutyCycle(double dutyCycle);

  public void setPosition(double position);
} 
