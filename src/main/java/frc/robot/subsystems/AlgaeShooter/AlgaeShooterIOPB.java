// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.AlgaeShooter;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.WoodbotConstants;
import frc.robot.subsystems.Elevator.ElevatorIO.ElevatorIOInputs;

public class AlgaeShooterIOPB implements AlgaeShooterIO {

   private final TalonFX algaeShooterMotor = new TalonFX(11, "Default Name"); // no ID

    private TalonFXConfiguration talonFXConfiguration = new TalonFXConfiguration();
    private MotorOutputConfigs outputConfigs = new MotorOutputConfigs();

  /** Creates a new AlgaeShooterIOWB. */
  public AlgaeShooterIOPB() {
    algaeShooterMotor.getConfigurator().apply((talonFXConfiguration));
    outputConfigs.withNeutralMode(NeutralModeValue.Brake);
    outputConfigs.withInverted(InvertedValue.Clockwise_Positive);
 
  }

public void updateInputs(AlgaeShooterIOInputs inputs){
  inputs.algaeShooterVoltage = algaeShooterMotor.getMotorVoltage().getValueAsDouble();
  inputs.algaeShooterPosition = algaeShooterMotor.getPosition().getValueAsDouble();
  inputs.algaeShooterVelocity = algaeShooterMotor.getVelocity().getValueAsDouble();
}


public void setDutyCycle(double dutyCycle) {
 algaeShooterMotor.set(dutyCycle);  

}
}

