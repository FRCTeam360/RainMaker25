// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.AlgaeShooter;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.EncoderConfig;
import com.revrobotics.spark.config.SparkFlexConfig;
import frc.robot.Constants;

public class AlgaeShooterIOCB implements AlgaeShooterIO {

  protected final SparkFlex algaeShooterMotorFront;
  protected final SparkFlex algaeShooterMotorBack;

  protected SparkFlexConfig frontConfig = new SparkFlexConfig();
  protected SparkFlexConfig backConfig = new SparkFlexConfig();
  protected final double positionConversionFactor = 1.0;

  /** Creates a new AlgaeShooterIOWB. */
  public AlgaeShooterIOCB() {
    this(
        new SparkFlex(Constants.CompBotConstants.ALGAE_SHOOTER_FRONT_ID, MotorType.kBrushless), 
        new SparkFlex(Constants.CompBotConstants.ALGAE_SHOOTER_BACK_ID, MotorType.kBrushless)
        );
  }

  public AlgaeShooterIOCB(SparkFlex algaeShooterMotorFront, SparkFlex algaeShooterMotorBack) {
    this.algaeShooterMotorFront = algaeShooterMotorFront;
    this.algaeShooterMotorBack = algaeShooterMotorBack;

    final double kP = 0.0;
    final double kI = 0.0;
    final double kD = 0.0;
    
    ClosedLoopConfig closedLoopConfig = new ClosedLoopConfig();
    closedLoopConfig.pid(kP, kI, kD);
    frontConfig.apply(closedLoopConfig);
    backConfig.apply(closedLoopConfig);
    EncoderConfig encoderConfig = new EncoderConfig();
    encoderConfig.positionConversionFactor(positionConversionFactor);
    frontConfig.apply(encoderConfig);
    backConfig.apply(encoderConfig);
    backConfig.follow(Constants.CompBotConstants.ALGAE_SHOOTER_FRONT_ID, true);
    backConfig.inverted(true);
    frontConfig.inverted(false);
    
    algaeShooterMotorFront.configure(frontConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    algaeShooterMotorBack.configure(backConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public void updateInputs(AlgaeShooterIOInputs inputs) {
    inputs.algaeShooterFrontVoltage = algaeShooterMotorFront.getBusVoltage();
    inputs.algaeShooterFrontPosition = algaeShooterMotorFront.getEncoder().getPosition();
    inputs.algaeShooterFrontVelocity = algaeShooterMotorFront.getEncoder().getVelocity();
    inputs.algaeShooterFromCurrent = algaeShooterMotorFront.getOutputCurrent();
    inputs.algaeShooterFromTemperature = algaeShooterMotorFront.getMotorTemperature();

    inputs.algaeShooterBackVoltage = algaeShooterMotorBack.getBusVoltage();
    inputs.algaeShooterBackPosition = algaeShooterMotorBack.getEncoder().getPosition();
    inputs.algaeShooterBackVelocity = algaeShooterMotorBack.getEncoder().getVelocity();
    inputs.algaeShooterBackCurrent = algaeShooterMotorBack.getOutputCurrent();
    inputs.algaeShooterBackTemperature = algaeShooterMotorBack.getMotorTemperature();
  }

  public void setDutyCycle(double dutyCycle) {
    algaeShooterMotorFront.set(dutyCycle);
  }

  public void setVelocity(double velocity) {
    algaeShooterMotorFront.getClosedLoopController().setReference(velocity, ControlType.kVelocity);
  }

  public void stop() {
    algaeShooterMotorFront.stopMotor();
  }
}
