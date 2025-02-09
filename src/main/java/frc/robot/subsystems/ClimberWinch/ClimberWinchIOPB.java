// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.ClimberWinch;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimberWinchIOPB implements ClimberWinchIO {

  private final SparkMax winchMotor = new SparkMax(10, MotorType.kBrushless);
  private final RelativeEncoder winchEncoder = winchMotor.getEncoder();
  
  private final SparkMaxConfig config = new SparkMaxConfig();

  /** Creates a new ClimberIOPB. */
  public ClimberWinchIOPB() {
    config.idleMode(IdleMode.kBrake);
    config.inverted(false);
    winchMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public void setDutyCycle(double dutyCycle) {
    winchMotor.set(dutyCycle);
  }

  public void updateInputs(ClimberWinchIOInputs inputs) {
    inputs.winchDutyCycle = winchMotor.getAppliedOutput();
    inputs.winchPosition = winchEncoder.getPosition();
    inputs.winchVelocity = winchEncoder.getVelocity();
  }

}
