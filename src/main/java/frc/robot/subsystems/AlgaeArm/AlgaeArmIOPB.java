// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.AlgaeArm;

import com.ctre.phoenix6.configs.MotorOutputConfigs;

import com.ctre.phoenix6.configs.TalonFXConfiguration;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class AlgaeArmIOPB implements AlgaeArmIO {

  private final TalonFX armMotor = new TalonFX(Constants.PracticeBotConstants.ALGAE_ARM_ID, "Default Name"); // placeholder ID

  private TalonFXConfiguration talonFXConfiguration = new TalonFXConfiguration();
  private MotorOutputConfigs outputConfigs = new MotorOutputConfigs();


  /** Creates a new AlgaeArmIOPB. */
  public AlgaeArmIOPB() {
    armMotor.getConfigurator().apply((talonFXConfiguration));
    outputConfigs.withNeutralMode(NeutralModeValue.Brake);
    outputConfigs.withInverted(InvertedValue.Clockwise_Positive);
  }

  public void setDutyCycle(double dutyCycle) {
    armMotor.set(dutyCycle);
  }

  public void setPosition(double position) {
    armMotor.setPosition(position);
  }
}
