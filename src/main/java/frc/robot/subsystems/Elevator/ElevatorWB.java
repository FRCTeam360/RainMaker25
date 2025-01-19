// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Elevator;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.wpilibj.DigitalInput;

import frc.robot.Constants;
import frc.robot.Constants.WoodbotConstants;
import frc.robot.subsystems.CoralIntake.CoralIntakeIO.CoralIntakeIOInputs;

/** Add your docs here. */
public class ElevatorWB implements ElevatorIO {

    private final TalonFX elevatorMotor = new TalonFX(WoodbotConstants.ELEVATOR_ID, "Default Name");

    private TalonFXConfiguration talonFXConfiguration = new TalonFXConfiguration();
    private MotorOutputConfigs outputConfigs = new MotorOutputConfigs();

    private final DigitalInput bottomSwitch = new DigitalInput(WoodbotConstants.ELEVATOR_BOTTOM_SWITCH);

    private final double GEAR_RATIO = 1.0;


    public ElevatorWB() {
        final double UPPER_LIMIT = 0.0;
        final double LOWER_LIMIT = 0.0;

        // TODO: add values
        final double kA = 0.0;
        final double kD = 0.0;
        final double kG = 0.65;
        final double kI = 0.0;
        final double kP = 5.0;
        final double kS = 0.05;
        final double kV = 0.0;

        Slot0Configs slot0Configs = talonFXConfiguration.Slot0;
        slot0Configs.kA = kA;
        slot0Configs.kD = kD;
        slot0Configs.kG = kG;
        slot0Configs.kI = kI;
        slot0Configs.kP = kP;
        slot0Configs.kS = kS;
        slot0Configs.kV = kV;

        final double motionMagicCruiseVelocity = 600.0;
        final double motionMagicAcceleration = 300.0;
        final double motionMagicCruiseJerk = 1000.0;

        elevatorMotor.getConfigurator().apply((talonFXConfiguration));
        outputConfigs.withNeutralMode(NeutralModeValue.Brake);
        outputConfigs.withInverted(InvertedValue.Clockwise_Positive);

        // talonFXConfiguration.SoftwareLimitSwitch.withForwardSoftLimitThreshold(UPPER_LIMIT);
        // talonFXConfiguration.SoftwareLimitSwitch.withForwardSoftLimitEnable(true);
        // talonFXConfiguration.SoftwareLimitSwitch.withReverseSoftLimitThreshold(LOWER_LIMIT);
        // talonFXConfiguration.SoftwareLimitSwitch.withReverseSoftLimitEnable(true);

        MotionMagicConfigs motionMagicConfigs = talonFXConfiguration.MotionMagic;
        motionMagicConfigs.MotionMagicCruiseVelocity = motionMagicCruiseVelocity;
        motionMagicConfigs.MotionMagicAcceleration = motionMagicAcceleration;
        motionMagicConfigs.MotionMagicJerk = motionMagicCruiseJerk;

        talonFXConfiguration.MotionMagic.withMotionMagicAcceleration(motionMagicAcceleration).withMotionMagicCruiseVelocity(motionMagicCruiseVelocity).withMotionMagicJerk(motionMagicCruiseJerk);
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

    /*
     * height is in motor rotations
     */
    public void setElevatorPostion(double height) {
        elevatorMotor.setPosition(height);
    }

    public boolean getBottomSwitch() {
        return !bottomSwitch.get();
    }
}
