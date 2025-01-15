// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Elevator;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.RelativeEncoder;

import frc.robot.Constants;
import frc.robot.subsystems.CoralIntake.CoralIntakeIO.CoralIntakeIOInputs;

/** Add your docs here. */
public class ElevatorWB implements ElevatorIO {

    private final TalonFX elevatorMotor = new TalonFX(Constants.ELEVATOR_ID, "Default Name");
    
    
    private final double woodElevatorKP = 0.0;
    private final double woodElevatorKI = 0.0;
    private final double woodElevatorKD = 0.0;
    private final double woodElevatorKF = 0.0;


    public ElevatorWB() {}

    public void updateInputs(ElevatorIOInputs inputs) {
        inputs.elevatorStatorCurrent = elevatorMotor.getStatorCurrent().getValueAsDouble();
        inputs.elevatorSupplyCurrent = elevatorMotor.getSupplyCurrent().getValueAsDouble();
        inputs.elevatorVoltage = elevatorMotor.getMotorVoltage().getValueAsDouble();
        inputs.elevatorPosition = elevatorMotor.getPosition().getValueAsDouble();
        inputs.elevatorVelocity = elevatorMotor.getVelocity().getValueAsDouble();
    }

    public void setDutyCycle(double dutyCycle) {
        elevatorMotor.set(dutyCycle);
    }

    public void setElevatorPostion(double height) {
        elevatorMotor.setPosition(height);
    }


}
