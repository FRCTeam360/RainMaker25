// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Climber;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimberIOPB implements ClimberIO {

  private final SparkMax bottomMotor = new SparkMax(10, MotorType.kBrushless);
  private final SparkMax topMotor = new SparkMax(11, MotorType.kBrushless);
  
  /** Creates a new ClimberIOPB. */
  public ClimberIOPB() {}

}
