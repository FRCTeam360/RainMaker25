// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.AlgaeRoller;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public interface AlgaeRollerIO {
  /** Creates a new AlgaeIntakeRollerIO. */
  @AutoLog
  public static class AlgaeRollerIOInputs {
    public double rollerPosition = 0.0;
    public double rollerVelocity = 0.0;
    public double rollerDutyCycle = 0.0;
    public double rollerCurrent = 0.0;
  }

  public void setDutyCycle(double dutyCycle);

  public default void updateInputs(AlgaeRollerIOInputs inputs) {}
}
