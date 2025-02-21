// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.AlgaeIntakeRoller;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class AlgaeIntakeRollerIOPB implements AlgaeIntakeRollerIO {
  private final SparkMax motor = new SparkMax(Constants.PracticeBotConstants.ALGAE_ROLLER, MotorType.kBrushless);
  private final RelativeEncoder encoder = motor.getEncoder();
  
  /** Creates a new AlgaeIntakeRollerIOPB. */
  public AlgaeIntakeRollerIOPB() {}

  public void setDutyCycle(double dutyCycle) {
    motor.set(dutyCycle);
  }

  public void updateInputs(AlgaeIntakeRollerIOInputs inputs) {
    inputs.rollerDutyCycle = motor.get();
    inputs.rollerPosition = encoder.getPosition();
    inputs.rollerVelocity = encoder.getVelocity();
  }
}
