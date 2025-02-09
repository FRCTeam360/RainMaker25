// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.ClimberWheel;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.ClimberWheel.ClimberWheelIO.ClimberWheelIOInputs;

public class ClimberWheelIOPB extends SubsystemBase {
  private final SparkMax wheelMotor = new SparkMax(16, MotorType.kBrushless);
  private final RelativeEncoder encoder = wheelMotor.getEncoder();

  /** Creates a new ClimberWheelIOPB. */
  public ClimberWheelIOPB() {}

  public void setDutyCycle(double dutyCycle) {
    wheelMotor.set(dutyCycle);
  }

  public void updateInputs(ClimberWheelIOInputs inputs) {
    inputs.wheelDutyCycle = wheelMotor.get();
    inputs.wheelPosition = encoder.getPosition();
    inputs.wheelVelocity = encoder.getVelocity();
  }
}
