// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.CoralIntake;

import org.littletonrobotics.junction.AutoLog;


import edu.wpi.first.wpilibj2.command.SubsystemBase;

public interface CoralIntakeIO {
  /** Creates a new CoralIntakeIO. */

  @AutoLog
  public static class CoralIntakeIOInputs {
    public double intakeStatorCurrent = 0.0;
    public double intakeSupplyCurrent = 0.0;
    public double intakeVoltage = 0.0;
    public double intakeVelocity = 0.0;
    public double intakePosition = 0.0;
    public double intakeDutyCycle = 0.0;

    // insert inputs
  }

  public default void updateInputs(CoralIntakeIOInputs inputs) {}

  public void setDutyCycle(double outputVolts);

  public void setVelocity(double velocity);
}
