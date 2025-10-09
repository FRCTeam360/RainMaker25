// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.ClimberWinch;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.EncoderConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import frc.robot.Constants.CompBotConstants;

public class ClimberWinchIOCB implements ClimberWinchIO {

  protected SparkMax winchMotor;
  protected RelativeEncoder winchEncoder;

  protected final double kP = 0.2;
  protected final double kI = 0.0;
  protected final double kD = 0.0;
  
  protected final double positionConversionFactor = 1.0;
  protected final SparkMaxConfig config = new SparkMaxConfig();

  /** Creates a new ClimberIOPB. */
  public ClimberWinchIOCB() {
    winchMotor =  new SparkMax(CompBotConstants.CLIMBER_WINCH_ID, MotorType.kBrushless);
    winchEncoder = winchMotor.getEncoder();

    config.idleMode(IdleMode.kBrake);
    config.inverted(true);
    ClosedLoopConfig closedLoopConfig = new ClosedLoopConfig();
    closedLoopConfig.pid(kP, kI, kD);
    config.apply(closedLoopConfig);
    EncoderConfig encoderConfig = new EncoderConfig();
    encoderConfig.positionConversionFactor(positionConversionFactor);
    config.apply(encoderConfig);
    winchMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public void setDutyCycle(double dutyCycle) {
    winchMotor.set(dutyCycle);
  }

  public void setPosition(double position) {
    winchMotor.getClosedLoopController().setReference(position, ControlType.kPosition);
  }

  public void updateInputs(ClimberWinchIOInputs inputs) {
    inputs.winchDutyCycle = winchMotor.getAppliedOutput();
    inputs.winchPosition = winchEncoder.getPosition();
    inputs.winchVelocity = winchEncoder.getVelocity();
    inputs.winchCurrent = winchMotor.getOutputCurrent();
    inputs.winchTemp = winchMotor.getMotorTemperature();
  }

}
