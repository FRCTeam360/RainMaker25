// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.AlgaeShooter;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.EncoderConfig;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.Elevator.ElevatorIO.ElevatorIOInputs;

public class AlgaeShooterIOPB implements AlgaeShooterIO {

  private final SparkFlex algaeShooterMotorFront = new SparkFlex(Constants.PracticeBotConstants.ALGAE_SHOOTER_FRONT_ID, MotorType.kBrushless); // no ID
  private final SparkFlex algaeShooterMotorBack = new SparkFlex(Constants.PracticeBotConstants.ALGAE_SHOOTER_BACK_ID, MotorType.kBrushless); // no ID

  private SparkFlexConfig frontConfig = new SparkFlexConfig();
  private SparkFlexConfig backConfig = new SparkFlexConfig();
  private final double positionConversionFactor = 1.0;

  /** Creates a new AlgaeShooterIOWB. */
  public AlgaeShooterIOPB() {
    // TODO: add values
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
    backConfig.follow(Constants.PracticeBotConstants.ALGAE_SHOOTER_FRONT_ID, true);
    backConfig.inverted(false);
    frontConfig.inverted(true);
    
    algaeShooterMotorFront.configure(frontConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    algaeShooterMotorBack.configure(backConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public void updateInputs(AlgaeShooterIOInputs inputs) {
    inputs.algaeShooterVoltage = algaeShooterMotorFront.getBusVoltage();
    inputs.algaeShooterPosition = algaeShooterMotorFront.getEncoder().getPosition();
    inputs.algaeShooterVelocity = algaeShooterMotorFront.getEncoder().getVelocity();
    inputs.algaeShooterCurrent = algaeShooterMotorFront.getOutputCurrent();
    inputs.algaeShooterTemperature = algaeShooterMotorFront.getMotorTemperature();
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
