// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.ClimberWheel;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import frc.robot.Constants.PracticeBotConstants;

public class ClimberWheelIOPB extends ClimberWheelIOCB {
  
  /** Creates a new ClimberWheelIOPB. */
  public ClimberWheelIOPB() {
    super();
    wheelMotor = new SparkMax(PracticeBotConstants.CLIMBER_ROLLER_ID, MotorType.kBrushless);

    config.idleMode(IdleMode.kBrake);
    config.inverted(false);
    wheelMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }
}