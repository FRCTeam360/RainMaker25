// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.CoralOuttake;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import frc.robot.Constants;
import frc.robot.subsystems.CoralOuttake.CoralOuttakeIO.CoralOuttakeIOInputs;

/** Add your docs here. */
public class CoralOuttakeWB implements CoralOuttakeIO {

    private final SparkMax outtakeMotor = new SparkMax(Constants.CORAL_OUTTAKE_ID, MotorType.kBrushless);
    private final RelativeEncoder encoder = outtakeMotor.getEncoder();

    private final double woodCoralOuttakeKP = 0.0;
    private final double woodCoralOuttakeKI = 0.0;
    private final double woodCoralOuttakeKD = 0.0;
    private final double woodCoralOuttakeKF = 0.0;

    public CoralOuttakeWB() {
    }

    public void setVelocity(double velocity) {
        System.out.println(":(");
    }
    
    public void setDutyCycle(double dutyCycle) {
        outtakeMotor.set(dutyCycle);
    }

    public void updateInputs(CoralOuttakeIOInputs inputs) {
        inputs.outtakeStatorCurrent = outtakeMotor.getOutputCurrent();
        inputs.outtakePosition = encoder.getPosition();
        inputs.outtakeVelocity = encoder.getVelocity();
        inputs.outtakeVoltage = outtakeMotor.getAppliedOutput() * outtakeMotor.getBusVoltage();
    }
}
