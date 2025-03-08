// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.AlgaeArm;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.configs.MotorOutputConfigs;

import com.ctre.phoenix6.configs.TalonFXConfiguration;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.EncoderConfig;
import com.revrobotics.spark.config.SoftLimitConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class AlgaeArmIOCB implements AlgaeArmIO {

  private final SparkMax armMotor = new SparkMax(Constants.CompBotConstants.ALGAE_ARM_ID, MotorType.kBrushless); // placeholder                                                                                                                    // ID
  private final RelativeEncoder encoder = armMotor.getEncoder();
  
  private final double kP = 0.025;
  private final double kI = 0.0;
  private final double kD = 0.0;

  private final double POSITION_CONVERSION_FACTOR = (1.0 / 5.0) * (1.0 / 5.0) * (18.0 / 36.0) * (360.0 / 1.0);
  private final double VELOCITY_CONVERSION_FACTOR = POSITION_CONVERSION_FACTOR / 60.0;

  private final double FORWARD_LIMIT = 150.0;
  private final double REVERSE_LIMIT = 10.0;
  private final SparkMaxConfig sparkMaxConfig = new SparkMaxConfig();

  private final double MAX_OUTPUT = 0.5;

  /** Creates a new AlgaeArmIOPB. */
  public AlgaeArmIOCB() {
    sparkMaxConfig.idleMode(IdleMode.kBrake);
    sparkMaxConfig.inverted(false);

    ClosedLoopConfig closedLoopConfig = new ClosedLoopConfig();
    closedLoopConfig.pid(kP, kI, kD);
    closedLoopConfig.minOutput(-MAX_OUTPUT);
    closedLoopConfig.maxOutput(MAX_OUTPUT);
    sparkMaxConfig.apply(closedLoopConfig);

    EncoderConfig encoderConfig = new EncoderConfig();
    encoderConfig.positionConversionFactor(POSITION_CONVERSION_FACTOR);
    encoderConfig.velocityConversionFactor(VELOCITY_CONVERSION_FACTOR);
    sparkMaxConfig.apply(encoderConfig);

    SoftLimitConfig softLimitConfig = new SoftLimitConfig();
    softLimitConfig.forwardSoftLimit(FORWARD_LIMIT);
    softLimitConfig.forwardSoftLimitEnabled(true);
    softLimitConfig.reverseSoftLimit(REVERSE_LIMIT);
    softLimitConfig.reverseSoftLimitEnabled(true);
    sparkMaxConfig.apply(softLimitConfig);

    armMotor.configure(sparkMaxConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public void updateInputs(AlgaeArmIOInputs inputs) {
    inputs.algaeArmAngle = encoder.getPosition();
    inputs.algaeArmVelocity = encoder.getVelocity();
    inputs.algaeArmVoltage = armMotor.getBusVoltage() * armMotor.getAppliedOutput();
    inputs.algaeArmCurrent = armMotor.getOutputCurrent();
    inputs.algaeArmTemp = armMotor.getMotorTemperature();
  }

  public void setDutyCycle(double dutyCycle) {
    armMotor.set(dutyCycle);
  }

  public void setPosition(double position) {
    armMotor.getClosedLoopController().setReference(position, ControlType.kPosition);
  }

  public void enableReverseSoftLimit(boolean enabled){
    sparkMaxConfig.softLimit.reverseSoftLimitEnabled(enabled);
  }

  /**
   * method for updating the encoder position
   * @param value new encoder position in motor rotations
   */
  public void setEncoder(double value) {
    encoder.setPosition(value);
  }
}