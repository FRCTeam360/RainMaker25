// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.AlgaeShooter;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.Elevator.ElevatorIO.ElevatorIOInputs;

public class AlgaeShooterIOPB implements AlgaeShooterIO {

  private final TalonFX algaeShooterMotor = new TalonFX(Constants.PracticeBotConstants.ALGAE_SHOOTER_ID,
      "Default Name"); // no ID

  private TalonFXConfiguration talonFXConfiguration = new TalonFXConfiguration();
  private MotorOutputConfigs outputConfigs = new MotorOutputConfigs();

  /** Creates a new AlgaeShooterIOWB. */
  public AlgaeShooterIOPB() {
    algaeShooterMotor.getConfigurator().apply((talonFXConfiguration));
    outputConfigs.withNeutralMode(NeutralModeValue.Brake);
    outputConfigs.withInverted(InvertedValue.Clockwise_Positive);

    // TODO: add values
    final double kA = 0.0;
    final double kD = 0.0;
    final double kG = 0.0;
    final double kI = 0.0;
    final double kP = 0.0;
    final double kS = 0.0;
    final double kV = 0.0;

    Slot0Configs slot0Configs = talonFXConfiguration.Slot0;
    slot0Configs.kA = kA;
    slot0Configs.kD = kD;
    slot0Configs.kG = kG;
    slot0Configs.kI = kI;
    slot0Configs.kP = kP;
    slot0Configs.kS = kS;
    slot0Configs.kV = kV;

    // TODO: values are for elevator, not sure if they apply the same to the shooter
    final double motionMagicAcceleration = 400.0;
    final double motionMagicCruiseVelocity = 85.0;
    final double motionMagicCruiseJerk = 1750.0;

    MotionMagicConfigs motionMagicConfigs = talonFXConfiguration.MotionMagic;
    motionMagicConfigs.MotionMagicCruiseVelocity = motionMagicCruiseVelocity;
    motionMagicConfigs.MotionMagicAcceleration = motionMagicAcceleration;
    motionMagicConfigs.MotionMagicJerk = motionMagicCruiseJerk;

    talonFXConfiguration.MotionMagic.withMotionMagicAcceleration(motionMagicAcceleration)
        .withMotionMagicCruiseVelocity(motionMagicCruiseVelocity).withMotionMagicJerk(motionMagicCruiseJerk);

  }

  public void updateInputs(AlgaeShooterIOInputs inputs) {
    inputs.algaeShooterVoltage = algaeShooterMotor.getMotorVoltage().getValueAsDouble();
    inputs.algaeShooterPosition = algaeShooterMotor.getPosition().getValueAsDouble();
    inputs.algaeShooterVelocity = algaeShooterMotor.getVelocity().getValueAsDouble();
  }

  public void setDutyCycle(double dutyCycle) {
    algaeShooterMotor.set(dutyCycle);
  }

  public void setVelocity(double velocity) {
    final VelocityVoltage velocityRequest = new VelocityVoltage(0).withSlot(0);
    algaeShooterMotor.setControl(velocityRequest.withVelocity(velocity));
  }
}
