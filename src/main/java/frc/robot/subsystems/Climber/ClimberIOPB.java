// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Climber;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimberIOPB implements ClimberIO {

  private final SparkMax winchMotor = new SparkMax(10, MotorType.kBrushless);
  private final SparkMax wheelMotor = new SparkMax(11, MotorType.kBrushless);
  
  private final RelativeEncoder winchEncoder = winchMotor.getEncoder();
  private final RelativeEncoder wheelEncoder = wheelMotor.getEncoder();
  
  private final SparkMaxConfig config = new SparkMaxConfig();

  /** Creates a new ClimberIOPB. */
  public ClimberIOPB() {
    config.idleMode(IdleMode.kBrake);
    config.inverted(false);
    winchMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    wheelMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public void setWinchDutyCycle(double dutyCycle) {
    winchMotor.set(dutyCycle);
  }

  public void setWheelDutyCycle(double dutyCycle) {
    wheelMotor.set(dutyCycle);
  }

  public void updateInputs(ClimberIOInputs inputs) {
    inputs.winchDutyCycle = winchMotor.getAppliedOutput();
    inputs.winchPosition = winchEncoder.getPosition();
    inputs.winchVelocity = winchEncoder.getVelocity();

    inputs.wheelDutyCycle = wheelMotor.getAppliedOutput();
    inputs.wheelPosition = wheelEncoder.getPosition();
    inputs.wheelVelocity = wheelEncoder.getVelocity();
  }

}
