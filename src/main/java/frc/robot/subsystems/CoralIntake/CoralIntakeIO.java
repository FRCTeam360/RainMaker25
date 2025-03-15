// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.CoralIntake;

import org.littletonrobotics.junction.AutoLog;

/** Add your docs here. */
public interface CoralIntakeIO {

  @AutoLog
  public static class CoralIntakeIOInputs{
    public double statorCurrent = 0.0;
    public double supplyCurrent = 0.0;
    public double voltage = 0.0;
    public double velocity = 0.0;
    public double position = 0.0;
    public double temperature = 0.0;
  }

  public default void updateInputs(CoralIntakeIOInputs inputs) {}

  /**
   * sets the speed ot a number between -1 and 1
   * @param dutyCycle
   */
  public void setDutyCycle(double dutyCycle);

 
  public void stop();
}
