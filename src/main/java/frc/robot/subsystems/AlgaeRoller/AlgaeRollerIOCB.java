// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.AlgaeRoller;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class AlgaeRollerIOCB implements AlgaeRollerIO {
  private final SparkMax motor = new SparkMax(Constants.CompBotConstants.ALGAE_ROLLER, MotorType.kBrushless);
  private final SparkMaxConfig config = new SparkMaxConfig();
  private final RelativeEncoder encoder = motor.getEncoder();

  /** Creates a new AlgaeIntakeRollerIOPB. */
  public AlgaeRollerIOCB() {
    config.inverted(true);
    config.idleMode(IdleMode.kCoast);
    config.smartCurrentLimit(20, 5);

    motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public void setDutyCycle(double dutyCycle) {
    motor.set(dutyCycle);
  }

  public void updateInputs(AlgaeRollerIOInputs inputs) {
    inputs.rollerDutyCycle = motor.get();
    inputs.rollerPosition = encoder.getPosition();
    inputs.rollerVelocity = encoder.getVelocity();
    inputs.rollerCurrent = motor.getAppliedOutput();
  }
}
