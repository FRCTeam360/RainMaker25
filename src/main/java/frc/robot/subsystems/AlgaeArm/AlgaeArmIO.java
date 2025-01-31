// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.AlgaeArm;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public interface AlgaeArmIO {
  /** Creates a new AlgeaArmIO. */
  @AutoLog
  public static class AlgaeArmIOInputs {
    public double algaeArmStatorCurrent = 0.0;
    // public double algaeArmSupplyCurrent = 0.0;
    public double algaeArmVoltage = 0.0;
    public double algaeArmVelocity = 0.0;
    public double algaeArmPosition = 0.0;
  }

  public default void updateInputs(AlgaeArmIOInputs inputs) {}

  public void setDutyCycle(double dutyCycle);
} 
