// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.AlgaeIntakeArm;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public interface AlgaeIntakeArmIO {
  /** Creates a new AlgaeIntakeIO. */
  @AutoLog
  public static class AlgaeIntakeArmIOInputs {
    public double armPosition = 0.0;
    public double armDutyCycle = 0.0;
    public double armVelocity = 0.0;
  }

  public void setDutyCycle(double dutyCycle);

  public default void updateInputs(AlgaeIntakeArmIOInputs inputs) {}
}
