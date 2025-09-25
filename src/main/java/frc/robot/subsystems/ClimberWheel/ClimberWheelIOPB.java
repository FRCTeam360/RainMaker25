// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.ClimberWheel;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PracticeBotConstants;
import frc.robot.subsystems.ClimberWheel.ClimberWheelIO.ClimberWheelIOInputs;

public class ClimberWheelIOPB extends ClimberWheelIOCB {
  
  /** Creates a new ClimberWheelIOPB. */
  public ClimberWheelIOPB() {
    wheelMotor = new SparkMax(PracticeBotConstants.CLIMBER_ROLLER_ID, MotorType.kBrushless);

    config.idleMode(IdleMode.kBrake);
    config.inverted(false);
    wheelMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }
}
