// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Elevator;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.mechanisms.DifferentialMechanism;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycle;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.Constants.PracticeBotConstants;
import frc.robot.Constants.WoodbotConstants;
import frc.robot.subsystems.CoralIntake.CoralIntakeIO.CoralIntakeIOInputs;

/** Add your docs here. */
public class ElevatorIOPB implements ElevatorIO {
    private final TalonFX backElevatorMotor = new TalonFX(PracticeBotConstants.BACK_ELEVATOR_ID, PracticeBotConstants.CANBUS_NAME);
    private final TalonFX frontElevatorMotor = new TalonFX(PracticeBotConstants.FRONT_ELEVATOR_ID, PracticeBotConstants.CANBUS_NAME);
    private final DifferentialMechanism elevatorDiff;
    private TalonFXConfiguration talonFXConfiguration = new TalonFXConfiguration();
    private MotorOutputConfigs outputConfigs = new MotorOutputConfigs();

    private final DigitalInput bottomSwitch = new DigitalInput(
        WoodbotConstants.ELEVATOR_BOTTOM_SWITCH
    );

    private final double GEAR_RATIO = 1.0;

    public ElevatorIOPB() {
        final double UPPER_LIMIT = 31.0;
        final double LOWER_LIMIT = 0.0;

        final double kA = 0.01;
        final double kD = 0.0;
        final double kG = 0.3;
        final double kI = 0.0;
        final double kP = 1.0; //5 original
        final double kS = 0.01;
        final double kV = 0.07;
        Slot0Configs slot0Configs = talonFXConfiguration.Slot0;
        slot0Configs.kA = kA;
        slot0Configs.kD = kD;
        slot0Configs.kG = kG;
        slot0Configs.kI = kI;
        slot0Configs.kP = kP;
        slot0Configs.kS = kS;
        slot0Configs.kV = kV;

        final double motionMagicCruiseVelocity = 800.0;
        final double motionMagicAcceleration = 350.0; //used to be 300 - jan 30
        final double motionMagicCruiseJerk = 1500.0;

        backElevatorMotor.getConfigurator().apply((talonFXConfiguration));
        frontElevatorMotor.getConfigurator().apply((talonFXConfiguration));

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
        
        
        elevatorDiff = new DifferentialMechanism(backElevatorMotor, frontElevatorMotor, false);
        backElevatorMotor.getConfigurator().apply(talonFXConfiguration, 0.05);
        backElevatorMotor.setNeutralMode(NeutralModeValue.Brake);
        frontElevatorMotor.setNeutralMode(NeutralModeValue.Brake);
        frontElevatorMotor.getConfigurator().apply(talonFXConfiguration, 0.05);
        elevatorDiff.applyConfigs();
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
        DutyCycleOut duty = new DutyCycleOut(dutyCycle);
        PositionDutyCycle positionDuty = new PositionDutyCycle(0); // difference between mechanism position should be zero?
        elevatorDiff.setControl(duty , positionDuty);
    }

    public void stop() {
        backElevatorMotor.stopMotor();
        frontElevatorMotor.stopMotor();
    }
    
    /*
     * value is new encoder value in rotations
     */
    public void setEncoder(double value) {
        backElevatorMotor.setPosition(value);
        frontElevatorMotor.setPosition(value);
    }

    /*
     * height is in motor rotations
     */
    public void setElevatorPostion(double height) {
        MotionMagicVoltage motionMagicVoltage = new MotionMagicVoltage(height);
        PositionVoltage positionVoltage = new PositionVoltage(0); // difference between mechanism position should be zero?
        elevatorDiff.setControl(motionMagicVoltage, positionVoltage);
    }

}
