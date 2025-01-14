// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Elevator;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase {
  private final ElevatorIO io;
  private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();
  
  /** Creates a new Elevator. */
  public Elevator(ElevatorIO io) {
    this.io = io;
  }

  public double getVelocity() {
    return inputs.elevatorVelocity;
  }

  public double getElevatorPosition() {
    return inputs.elevatorPosition;
  }

  public void setFF(double ff) {
    io.setFF(ff);
  }

  public void setElevatorPostion(double height) {
    io.setElevatorPostion(height);
  }

  public void setDutyCycle(double percent) {
    io.setDutyCycle(percent);
  }

  public void runElevator(double speed) {
    io.runElevator(speed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    io.updateInputs(inputs);
    Logger.processInputs("Elevator", inputs);
  }
}