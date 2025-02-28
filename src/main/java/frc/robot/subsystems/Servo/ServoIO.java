// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Servo;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public interface ServoIO {
  /** Creates a new ServoIO. */
  @AutoLog
  public static class ServoIOInputs {
    public double setOutput = 0.0;
}

  public default void updateInputs(ServoIOInputs inputs){}

  public void setServoSpeed(double speed);
}
