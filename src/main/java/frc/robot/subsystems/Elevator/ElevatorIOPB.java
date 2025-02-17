// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Elevator;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.Constants.PracticeBotConstants;
import frc.robot.Constants.WoodbotConstants;
import frc.robot.subsystems.CoralIntake.CoralIntakeIO.CoralIntakeIOInputs;

/** Add your docs here. */
public class ElevatorIOPB implements ElevatorIO {
    private final TalonFX backElevatorMotor = new TalonFX(PracticeBotConstants.BACK_ELEVATOR_ID, PracticeBotConstants.CANBUS_NAME);
    private final TalonFX frontElevatorMotor = new TalonFX(PracticeBotConstants.FRONT_ELEVATOR_ID, PracticeBotConstants.CANBUS_NAME);

    private TalonFXConfiguration talonFXConfiguration = new TalonFXConfiguration();
    private MotorOutputConfigs outputConfigs = new MotorOutputConfigs();

    private final DigitalInput bottomSwitch = new DigitalInput(
        WoodbotConstants.ELEVATOR_BOTTOM_SWITCH
    );

    private final double GEAR_RATIO = 1.0;

    public ElevatorIOPB() {
        final double UPPER_LIMIT = 0;
        final double LOWER_LIMIT = 0;

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
        final double motionMagicAcceleration = 200.0; //used to be 300 - jan 30
        final double motionMagicCruiseJerk = 1000.0;

        backElevatorMotor.getConfigurator().apply((talonFXConfiguration));
        //outputConfigs.withInverted(InvertedValue.Clockwise_Positive);

        // talonFXConfiguration.SoftwareLimitSwitch.withForwardSoftLimitThreshold(UPPER_LIMIT);
        // talonFXConfiguration.SoftwareLimitSwitch.withForwardSoftLimitEnable(true);
        // talonFXConfiguration.SoftwareLimitSwitch.withReverseSoftLimitThreshold(LOWER_LIMIT);
        // talonFXConfiguration.SoftwareLimitSwitch.withReverseSoftLimitEnable(true);

        MotionMagicConfigs motionMagicConfigs = talonFXConfiguration.MotionMagic;

        motionMagicConfigs.MotionMagicCruiseVelocity = motionMagicCruiseVelocity;
        motionMagicConfigs.MotionMagicAcceleration = motionMagicAcceleration;
        motionMagicConfigs.MotionMagicJerk = motionMagicCruiseJerk;

        talonFXConfiguration
            .MotionMagic.withMotionMagicAcceleration(motionMagicAcceleration)
            .withMotionMagicCruiseVelocity(motionMagicCruiseVelocity)
            .withMotionMagicJerk(motionMagicCruiseJerk);

        talonFXConfiguration.MotorOutput = outputConfigs;

        backElevatorMotor.getConfigurator().apply(talonFXConfiguration, 0.05);
        backElevatorMotor.setNeutralMode(NeutralModeValue.Brake);
        frontElevatorMotor.setControl(new Follower(PracticeBotConstants.BACK_ELEVATOR_ID, true));

    }

    public void updateInputs(ElevatorIOInputs inputs) {
        inputs.elevatorStatorCurrent = backElevatorMotor.getStatorCurrent().getValueAsDouble();
        inputs.elevatorSupplyCurrent = backElevatorMotor.getSupplyCurrent().getValueAsDouble();
        inputs.elevatorVoltage = backElevatorMotor.getMotorVoltage().getValueAsDouble();
        inputs.elevatorPosition = backElevatorMotor.getPosition().getValueAsDouble();
        inputs.elevatorVelocity = backElevatorMotor.getVelocity().getValueAsDouble();
        inputs.elevatorSensor = !bottomSwitch.get();
    }

    public void setDutyCycle(double dutyCycle) {
        backElevatorMotor.set(dutyCycle);
    }

    public void stop() {
        backElevatorMotor.stopMotor();
    }
    
    /*
     * value is new encoder value in rotations
     */
    public void setEncoder(double value) {
        backElevatorMotor.setPosition(value);
    }

    /*
     * height is in motor rotations
     */
    public void setElevatorPostion(double height) {
        MotionMagicVoltage motionMagicVoltage = new MotionMagicVoltage(height);
        backElevatorMotor.setControl(motionMagicVoltage);
    }

}
