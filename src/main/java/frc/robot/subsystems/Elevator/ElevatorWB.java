// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Elevator;

import com.ctre.phoenix6.hardware.TalonFX;

import frc.robot.Constants;

/** Add your docs here. */
public class ElevatorWB implements ElevatorIO {

    private final TalonFX elevatorMotor = new TalonFX(Constants.ELEVATOR_ID, "Default Name");
    
    private final double woodElevatorKP = 0.0;
    private final double woodElevatorKI = 0.0;
    private final double woodElevatorKD = 0.0;
    private final double woodElevatorKF = 0.0;

    private final double practiceElevatorKP = 0.0;
    private final double practiceElevatorKI = 0.0;
    private final double practiceElevatorKD = 0.0;
    private final double practiceElevatorKF = 0.0;

    public ElevatorWB() {

    }

    public void updateInputs() {

    }
}
