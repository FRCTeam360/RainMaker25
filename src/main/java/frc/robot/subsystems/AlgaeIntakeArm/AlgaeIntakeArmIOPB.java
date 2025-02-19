// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.AlgaeIntakeArm;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AlgaeIntakeArmIOPB implements AlgaeIntakeArmIO {
  private final SparkMax motor = new SparkMax(13, MotorType.kBrushless);
  private final AbsoluteEncoder encoder =  motor.getAbsoluteEncoder();

  /** Creates a new AlgaeIntakeIOPB. */
  public AlgaeIntakeArmIOPB() {}

  public void setDutyCycle(double dutyCycle) {
    motor.set(dutyCycle);
  }

  public void updateInputs(AlgaeIntakeArmIOInputs inputs) {
    inputs.armDutyCycle = motor.get();
    inputs.armPosition = encoder.getPosition();
    inputs.armVelocity = encoder.getVelocity(); 
  }
}
