// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.CoralIntake;

import com.ctre.phoenix6.hardware.TalonFX;
import frc.robot.Constants;
import frc.robot.Constants.WoodbotConstants;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.sim.SparkRelativeEncoderSim;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkRelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;


/** Add your docs here. */
public class CoralIntakeWB implements CoralIntakeIO {
    
    private final SparkMax intakeMotor = new SparkMax(WoodbotConstants.CORAL_INTAKE_ID, MotorType.kBrushless);
    private final RelativeEncoder encoder = intakeMotor.getEncoder();
    private final SparkMaxConfig sparkMaxConfig = new SparkMaxConfig();
    
    private final double KP = 0.0;
    private final double KI = 0.0;
    private final double KD = 0.0;
    private final double KF = 0.0;

    private final double GEAR_RATIO = 1.0;

    public CoralIntakeWB() {
        sparkMaxConfig.idleMode(IdleMode.kBrake);
        sparkMaxConfig.inverted(false);
        intakeMotor.configure(sparkMaxConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void setDutyCycle(double dutyCycle) {
        intakeMotor.set(dutyCycle);
    }

    public void setVelocity(double velocity) {
        System.out.println(":(");
    }

    public void updateInputs(CoralIntakeIOInputs inputs) {
        inputs.intakeVelocity = encoder.getVelocity();
        inputs.intakePosition = encoder.getPosition();
        inputs.intakeStatorCurrent = intakeMotor.getOutputCurrent();
        inputs.intakeVoltage = intakeMotor.getAppliedOutput() * intakeMotor.getBusVoltage();
        inputs.intakeDutyCycle = intakeMotor.getAppliedOutput();
    } 
}
