// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.AlgaeIntake;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AlgaeIntakeIOPB implements AlgaeIntakeIO {
  private final  motor = new 
  /** Creates a new AlgaeIntakeIOPB. */
  public AlgaeIntakeIOPB() {}

  public void setDutyCycle(double dutyCycle) {
    motor.set(dutyCycle);
  }

  public void updateInputs(AlgaeIntakeIOInputs inputs) {
    inputs.intakeDutyCycle = motor.getDutyCycle();
    inputs.intakePosition = encoder.getPosition();
    inputs.intakeVelocity = encoder.getVelocity(); 
  }
}
