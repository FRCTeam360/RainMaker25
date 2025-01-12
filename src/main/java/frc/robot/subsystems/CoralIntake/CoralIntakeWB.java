// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.CoralIntake;

import com.ctre.phoenix6.hardware.TalonFX;
import frc.robot.Constants;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;


/** Add your docs here. */
public class CoralIntakeWB implements CoralIntakeIO {
    
    private final SparkMax coralIntake = new SparkMax(Constants.CORAL_INTAKE_ID, MotorType.kBrushless);
    
    private final double woodCoralIntakeKP = 0.0;
    private final double woodCoralIntakeKI = 0.0;
    private final double woodCoralIntakeKD = 0.0;
    private final double woodCoralIntakeKF = 0.0;

    private final double coralIntakeKP = 0.0;
    private final double coralIntakeKI = 0.0;
    private final double coralIntakeKD = 0.0;
    private final double coralIntakeKF = 0.0;

    public CoralIntakeWB() {

    }

    public void updateInputs() {

    }
}
