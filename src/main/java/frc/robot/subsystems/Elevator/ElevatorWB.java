// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Elevator;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.RelativeEncoder;

import frc.robot.Constants;
import frc.robot.Constants.WoodbotConstants;
import frc.robot.subsystems.CoralIntake.CoralIntakeIO.CoralIntakeIOInputs;

/** Add your docs here. */
public class ElevatorWB implements ElevatorIO {

    private final TalonFX elevatorMotor = new TalonFX(Constants.ELEVATOR_ID, "Default Name");

    private TalonFXConfiguration talonFXConfiguration = new TalonFXConfiguration();
    
    private final double UPPER_LIMIT = WoodbotConstants.ELEVATOR_UPPER_LIMIT;
    private final double LOWER_LIMIT = WoodbotConstants.ELEVATOR_LOWER_LIMIT;
    
    private final double woodElevatorKP = WoodbotConstants.ELEVATOR_KP;
    private final double woodElevatorKI = WoodbotConstants.ELEVATOR_KI;
    private final double woodElevatorKD = WoodbotConstants.ELEVATOR_KD;
    private final double woodElevatorKF = WoodbotConstants.ELEVATOR_KF;


    public ElevatorWB() {
        talonFXConfiguration.SoftwareLimitSwitch.withForwardSoftLimitThreshold(UPPER_LIMIT);
        talonFXConfiguration.SoftwareLimitSwitch.withForwardSoftLimitEnable(true);
        talonFXConfiguration.SoftwareLimitSwitch.withReverseSoftLimitThreshold(LOWER_LIMIT);
        talonFXConfiguration.SoftwareLimitSwitch.withReverseSoftLimitEnable(true);
    }

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
