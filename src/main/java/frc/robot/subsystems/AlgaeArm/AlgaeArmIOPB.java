// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.AlgaeArm;

import com.ctre.phoenix6.configs.MotorOutputConfigs;

import com.ctre.phoenix6.configs.TalonFXConfiguration;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class AlgaeArmIOPB implements AlgaeArmIO {

  private final SparkMax armMotor = new SparkMax(Constants.PracticeBotConstants.ALGAE_ARM_ID, MotorType.kBrushless); // placeholder                                                                                                                    // ID
  private final RelativeEncoder encoder = armMotor.getEncoder();
  private final PIDController pid = new PIDController(0, 0, 0); // TODO: find pid values

  private final SparkMaxConfig sparkMaxConfig = new SparkMaxConfig();

  /** Creates a new AlgaeArmIOPB. */
  public AlgaeArmIOPB() {
    sparkMaxConfig.idleMode(IdleMode.kBrake);
    sparkMaxConfig.inverted(false);
    armMotor.configure(sparkMaxConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public void updateInputs(AlgaeArmIOInputs inputs) {
    inputs.algaeArmAngle = encoder.getPosition();
    inputs.algaeArmVoltage = armMotor.getBusVoltage() * armMotor.getAppliedOutput();
    inputs.algaeArmCurrent = armMotor.getOutputCurrent();
    inputs.algaeArmTemp = armMotor.getMotorTemperature();
  }

  public void setDutyCycle(double dutyCycle) {
    armMotor.set(dutyCycle);
  }

  public void setPosition(double position) {
    pid.setSetpoint(position);
  }
}