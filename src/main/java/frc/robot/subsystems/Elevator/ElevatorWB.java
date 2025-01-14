// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Elevator;

import com.ctre.phoenix6.hardware.TalonFX;

import frc.robot.Constants;
import frc.robot.subsystems.CoralIntake.CoralIntakeIO.CoralIntakeIOInputs;

/** Add your docs here. */
public class ElevatorWB implements ElevatorIO {

    private final TalonFX talonFX = new TalonFX(Constants.ELEVATOR_ID, "Default Name");
    
    private final double woodElevatorKP = 0.0;
    private final double woodElevatorKI = 0.0;
    private final double woodElevatorKD = 0.0;
    private final double woodElevatorKF = 0.0;


    public ElevatorWB() {

    }

    public void updateInputs(ElevatorIOInputs inputs) {
        inputs.elevatorStatorCurrent = talonFX.getStatorCurrent().getValueAsDouble();
        inputs.elevatorSupplyCurrent = talonFX.getSupplyCurrent().getValueAsDouble();
        inputs.elevatorVoltage = talonFX.getMotorVoltage().getValueAsDouble();
        inputs.elevatorPosition = talonFX.getPosition().getValueAsDouble();
        inputs.elevatorVelocity = talonFX.getVelocity().getValueAsDouble();
    }

    public void setDutyCycle(double percent) {
        
    }

    public double getElevatorPosition() {
        return 0.0; //placeholder
    }

    public void setElevatorPostion(double height) {

    }

    public double getVelocity() {
        return 0.0;
    }

    public void runElevator(double speed) {

    }

    public void setFF(double ff) {

    }
}
