// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.ClimberWheel;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PracticeBotConstants;
import frc.robot.subsystems.ClimberWheel.ClimberWheelIO.ClimberWheelIOInputs;

public class ClimberWheelIOPB implements ClimberWheelIO {
  private final SparkMax wheelMotor = new SparkMax(PracticeBotConstants.CLIMBER_WHEEL_ID, MotorType.kBrushless);
  private final RelativeEncoder encoder = wheelMotor.getEncoder();
  private final PIDController pid = new PIDController(0, 0, 0); // TODO: find pid values

  private final SparkMaxConfig config = new SparkMaxConfig();

  /** Creates a new ClimberWheelIOPB. */
  public ClimberWheelIOPB() {
    config.idleMode(IdleMode.kBrake);
    config.inverted(false);
    wheelMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public void setDutyCycle(double dutyCycle) {
    wheelMotor.set(dutyCycle);
  }

  public void setPosition(double position) {
    pid.setSetpoint(position);
  }

  public void updateInputs(ClimberWheelIOInputs inputs) {
    inputs.wheelDutyCycle = wheelMotor.get();
    inputs.wheelPosition = encoder.getPosition();
    inputs.wheelVelocity = encoder.getVelocity();
    inputs.wheelCurrent = wheelMotor.getOutputCurrent();
    inputs.wheelTemp = wheelMotor.getMotorTemperature();
  }
}
