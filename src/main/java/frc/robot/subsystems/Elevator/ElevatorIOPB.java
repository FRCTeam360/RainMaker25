// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Elevator;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants.PracticeBotConstants;
import frc.robot.Constants.WoodbotConstants;
import org.littletonrobotics.junction.Logger;

/** Add your docs here. */
public class ElevatorIOPB extends ElevatorIOCB {
  // private final DifferentialMechanism elevatorDiff;
  // private DifferentialSensorsConfigs sens = backConfig.DifferentialSensors;

  public ElevatorIOPB() {
    super(
        new TalonFX(PracticeBotConstants.BACK_ELEVATOR_ID, PracticeBotConstants.CANBUS_NAME),
        new TalonFX(PracticeBotConstants.FRONT_ELEVATOR_ID, PracticeBotConstants.CANBUS_NAME)
    );

    final double UPPER_LIMIT = 31.0;
    final double LOWER_LIMIT = 0.0;

    final double motionMagicCruiseVelocity = 800.0;
    final double motionMagicAcceleration = 350.0; // used to be 300 - jan 30
    final double motionMagicCruiseJerk = 1500.0;

    backElevatorMotor.getConfigurator().apply(new TalonFXConfiguration());
    frontElevatorMotor.getConfigurator().apply(new TalonFXConfiguration());

    // outputConfigs.withInverted(InvertedValue.Clockwise_Positive);

    // talonFXConfiguration.SoftwareLimitSwitch.withForwardSoftLimitThreshold(UPPER_LIMIT);
    // talonFXConfiguration.SoftwareLimitSwitch.withForwardSoftLimitEnable(true);
    // talonFXConfiguration.SoftwareLimitSwitch.withReverseSoftLimitThreshold(LOWER_LIMIT);
    // talonFXConfiguration.SoftwareLimitSwitch.withReverseSoftLimitEnable(true);

    MotionMagicConfigs motionMagicConfigs = backConfig.MotionMagic;

    motionMagicConfigs.MotionMagicCruiseVelocity = motionMagicCruiseVelocity;
    motionMagicConfigs.MotionMagicAcceleration = motionMagicAcceleration;
    motionMagicConfigs.MotionMagicJerk = motionMagicCruiseJerk;

    backConfig
        .MotionMagic
        .withMotionMagicAcceleration(motionMagicAcceleration)
        .withMotionMagicCruiseVelocity(motionMagicCruiseVelocity)
        .withMotionMagicJerk(motionMagicCruiseJerk);

    backConfig.MotorOutput = outputConfigs;

    // sens.withDifferentialTalonFXSensorID(frontElevatorMotor.getDeviceID());
    // sens.withDifferentialSensorSource(DifferentialSensorSourceValue.RemoteTalonFX_Diff);

    backElevatorMotor.setNeutralMode(NeutralModeValue.Brake);
    backConfig.MotorOutput.withInverted(InvertedValue.CounterClockwise_Positive);
    backElevatorMotor.getConfigurator().apply(backConfig, 0.05);

    frontElevatorMotor.setNeutralMode(NeutralModeValue.Brake);
    frontConfig.MotorOutput.withInverted(InvertedValue.CounterClockwise_Positive);
    frontElevatorMotor.getConfigurator().apply(frontConfig, 0.05);

    // elevatorDiff = new DifferentialMechanism(backElevatorMotor, frontElevatorMotor, false);
    // elevatorDiff.applyConfigs();
    frontElevatorMotor.setControl(new Follower(PracticeBotConstants.BACK_ELEVATOR_ID, true));
  }

  public void updateInputs(ElevatorIOInputs inputs) {
    inputs.elevatorStatorCurrent = backElevatorMotor.getStatorCurrent().getValueAsDouble();
    inputs.elevatorSupplyCurrent = backElevatorMotor.getSupplyCurrent().getValueAsDouble();
    inputs.elevatorVoltage = backElevatorMotor.getMotorVoltage().getValueAsDouble();
    inputs.elevatorPosition = backElevatorMotor.getPosition().getValueAsDouble();
    inputs.elevatorVelocity = backElevatorMotor.getVelocity().getValueAsDouble();
    inputs.elevatorSensor = !bottomSwitch.get();

    Logger.recordOutput("front motor", frontElevatorMotor.getPosition().getValueAsDouble());
    Logger.recordOutput("back motor", backElevatorMotor.getPosition().getValueAsDouble());

    Logger.recordOutput(
        "front motor duty cycle", frontElevatorMotor.getDutyCycle().getValueAsDouble());
    Logger.recordOutput(
        "back motor duty cycle", backElevatorMotor.getDutyCycle().getValueAsDouble());
  }

  public void setDutyCycle(double dutyCycle) {
    DutyCycleOut duty = new DutyCycleOut(dutyCycle);
    // Logger.recordOutput("duty", duty.Output);
    // DifferentialDutyCycle differentialDuty = new DifferentialDutyCycle(dutyCycle, 0.0); //
    // difference between mechanism position should be zero?
    // PositionDutyCycle positionDuty = new PositionDutyCycle(0.0);
    // elevatorDiff.setControl(duty, differentialDuty);
    // backElevatorMotor.set(dutyCycle);
    frontElevatorMotor.setControl(new Follower(PracticeBotConstants.BACK_ELEVATOR_ID, true));

    backElevatorMotor.setControl(duty);
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
    frontElevatorMotor.setControl(new Follower(PracticeBotConstants.BACK_ELEVATOR_ID, true));

    // PositionVoltage positionVoltage = new PositionVoltage(0); // difference between mechanism
    // position should be zero?
    // elevatorDiff.setControl(motionMagicVoltage, positionVoltage);
    backElevatorMotor.setControl(motionMagicVoltage);
  }
}
