// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.CoralIntake;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import frc.robot.Constants;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import com.revrobotics.spark.SparkMax;

/** Add your docs here. */
public class CoralIntakeIOPB implements CoralIntakeIO {

    private final SparkMax motor = new SparkMax(Constants.CompBotConstants.CORAL_SHOOTER_ID, MotorType.kBrushless);
    private final RelativeEncoder encoder = motor.getEncoder();
    private final SparkMaxConfig sparkMaxConfig = new SparkMaxConfig();
  
    private final double KP = 0.0;
    private final double KI = 0.0;
    private final double KD = 0.0;
    private final double KF = 0.0;

    CoralIntakeIOPB(){
        sparkMaxConfig.idleMode(IdleMode.kBrake);
        sparkMaxConfig.inverted(false);
        sparkMaxConfig.smartCurrentLimit(20, 5);

        motor.configure(sparkMaxConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public void updateInputs(CoralIntakeIOInputs inputs) {
        inputs.position = encoder.getPosition();
        inputs.velocity = encoder.getVelocity();

        inputs.statorCurrent = motor.getOutputCurrent();
        inputs.temperature = motor.getMotorTemperature();
        inputs.voltage = motor.getAppliedOutput() * motor.getBusVoltage();
    }

    public void setDutyCycle(double dutyCycle){
        motor.set(dutyCycle);
    }

    public void stop() {
        motor.set(0.0);
    }
}
