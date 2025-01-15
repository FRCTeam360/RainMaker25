// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.CoralIntake;

import com.ctre.phoenix6.hardware.TalonFX;
import frc.robot.Constants;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.sim.SparkRelativeEncoderSim;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkRelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;


/** Add your docs here. */
public class CoralIntakeWB implements CoralIntakeIO {
    
    private final SparkMax intakeMotor = new SparkMax(Constants.CORAL_INTAKE_ID, MotorType.kBrushless);
    private final RelativeEncoder encoder = intakeMotor.getEncoder();
    
    private final double woodCoralIntakeKP = 0.0;
    private final double woodCoralIntakeKI = 0.0;
    private final double woodCoralIntakeKD = 0.0;
    private final double woodCoralIntakeKF = 0.0;

    private final double GEAR_RATIO = 1.0;

    public CoralIntakeWB() {

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
