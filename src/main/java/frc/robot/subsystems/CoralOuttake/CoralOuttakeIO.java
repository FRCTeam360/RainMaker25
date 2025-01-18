// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.CoralOuttake;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public interface CoralOuttakeIO {
  /** Creates a new CoralOutakeIO. */
  
  @AutoLog
  public static class CoralOuttakeIOInputs {
    public double outtakeStatorCurrent = 0.0;
    public double outtakeSupplyCurrent = 0.0;
    public double outtakeVoltage = 0.0;
    public double outtakeVelocity = 0.0;
    public double outtakePosition = 0.0;
    // insert inputs
  }

  public default void updateInputs(CoralOuttakeIOInputs inputs) {}

  /**
   * sets the speed ot a number between -1 and 1
   * @param dutyCycle
   */
  public void setDutyCycle(double dutyCycle);

  public boolean getOuttakeSensor();  
  // insert subsystem methods
}
