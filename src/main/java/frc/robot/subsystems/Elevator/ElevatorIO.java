// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Elevator;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public interface ElevatorIO {
  /** Creates a new ElevatorIO. */

  @AutoLog
  public static class ElevatorIOInputs {
    public double elevatorStatorCurrent = 0.0;
    public double elevatorSupplyCurrent = 0.0;
    public double elevatorVoltage = 0.0;
    public double elevatorVelocity = 0.0;
    public double elevatorPosition = 0.0;
      // insert inputs
  }

  public default void updateInputs(ElevatorIOInputs inputs) {}

  public void setDutyCycle(double percent);

  public double getElevatorPosition();

  public double getVelocity();

  public void setFF(double ff);

  public void setElevatorPostion(double height);

  public void runElevator(double speed);



  // insert methods from subsystem layer
}
