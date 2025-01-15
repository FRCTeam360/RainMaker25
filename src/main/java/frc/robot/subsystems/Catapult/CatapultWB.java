// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Catapult;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class CatapultWB implements CatapultIO {

  private final SparkMax catapultMotor = new SparkMax(5, MotorType.kBrushless); // device id is a placeholder: CHANGE THIS!!!
  private final RelativeEncoder encoder = catapultMotor.getEncoder();

  
  /** Creates a new CatapultWB. */
  public CatapultWB() {}

  public void updateInputs(CatapultIOInputs inputs) {
      inputs.catapultPosition = encoder.getPosition();
      inputs.catapultStatorCurrent = catapultMotor.getOutputCurrent();
      inputs.catapultVelocity = encoder.getVelocity();
      inputs.catapultVoltage = catapultMotor.getAppliedOutput() * catapultMotor.getBusVoltage();
  }
}
