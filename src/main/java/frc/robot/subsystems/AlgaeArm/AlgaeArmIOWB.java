// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.AlgaeArm;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.AlgaeArm.AlgaeArmIO.AlgaeArmIOInputs;

public class AlgaeArmIOWB implements AlgaeArmIO {
  private final SparkMax armMotor = new SparkMax(2/*placeholder*/, MotorType.kBrushless);
  private final RelativeEncoder encoder = armMotor.getEncoder();
  private final SparkMaxConfig sparkMaxConfig = new SparkMaxConfig();


  /** Creates a new AlgaeArmIOWB. */
  public AlgaeArmIOWB() {
    sparkMaxConfig.idleMode(IdleMode.kBrake);
    sparkMaxConfig.inverted(false);
    armMotor.configure(sparkMaxConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public void updateInputs(AlgaeArmIOInputs inputs){
    inputs.algaeArmPosition = encoder.getPosition();
    inputs.algaeArmStatorCurrent = armMotor.getOutputCurrent();
    inputs.algaeArmVelocity = encoder.getVelocity();
    inputs.algaeArmVoltage = armMotor.getAppliedOutput() * armMotor.getBusVoltage();
  }

  public void setPosition(double position){
    encoder.setPosition(position);
  }
}
