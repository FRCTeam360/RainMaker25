// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.AlgaeIntakeArm;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AlgaeIntakeArmIOPB implements AlgaeIntakeArmIO {
  private final SparkMax motor = new SparkMax(13, MotorType.kBrushless);
  private final RelativeEncoder encoder = motor.getEncoder();
  
  /** Creates a new AlgaeIntakeIOPB. */
  public AlgaeIntakeArmIOPB() {}

  public void setDutyCycle(double dutyCycle) {
    motor.set(dutyCycle);
  }

  public void updateInputs(AlgaeIntakeArmIOInputs inputs) {
    inputs.intakeDutyCycle = motor.get();
    inputs.intakePosition = encoder.getPosition();
    inputs.intakeVelocity = encoder.getVelocity(); 
  }
}
