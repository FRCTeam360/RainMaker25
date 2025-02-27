// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.AlgaeTilt;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.AbsoluteEncoderConfig;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.EncoderConfig;
import com.revrobotics.spark.config.SoftLimitConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class AlgaeTiltIOCB implements AlgaeTiltIO {
  private final SparkMax motor = new SparkMax(Constants.CompBotConstants.ALGAE_TILT, MotorType.kBrushless);
  private final AbsoluteEncoder encoder = motor.getAbsoluteEncoder(); // TODO: make absolute when we get one!!

  private final double kP = 0.035 * 4.0;
  private final double kI = 0.0;
  private final double kD = 0.0;

  private final double forwardLimit = 38.0;
  private final double reverseLimit = -10.0;

  private final double ZERO_OFFSET = 0.0; // TODO: find the zero offset

  private final double positionConversionFactor = 1.0;
  private final SparkMaxConfig sparkMaxConfig = new SparkMaxConfig();

  /** Creates a new AlgaeIntakeIOPB. */
  public AlgaeTiltIOCB() {
    sparkMaxConfig.idleMode(IdleMode.kBrake);
    sparkMaxConfig.inverted(false);

    SoftLimitConfig softLimitConfig = new SoftLimitConfig();
    softLimitConfig.forwardSoftLimit(forwardLimit);
    softLimitConfig.forwardSoftLimitEnabled(true);
    softLimitConfig.reverseSoftLimit(reverseLimit);
    softLimitConfig.reverseSoftLimitEnabled(true);
    sparkMaxConfig.apply(softLimitConfig);

    ClosedLoopConfig closedLoopConfig = new ClosedLoopConfig();
    closedLoopConfig.pid(kP, kI, kD);

    sparkMaxConfig.apply(closedLoopConfig);

    EncoderConfig encoderConfig = new EncoderConfig();
    encoderConfig.positionConversionFactor(positionConversionFactor);
    
    AbsoluteEncoderConfig absoluteEncoderConfig = new AbsoluteEncoderConfig();
    absoluteEncoderConfig.zeroOffset(ZERO_OFFSET);

    sparkMaxConfig.apply(absoluteEncoderConfig);
    motor.configure(sparkMaxConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public void setDutyCycle(double dutyCycle) {
    motor.set(dutyCycle);
  }

  public void setPosition(double position) {
    motor.getClosedLoopController().setReference(position, ControlType.kPosition);
  }

  /**
   * method for updating the encoder value
   * 
   * @param value sets the new encoder value in rotations!!
   */

  public void updateInputs(AlgaeTiltIOInputs inputs) {
    inputs.armDutyCycle = motor.get();
    inputs.armPosition = encoder.getPosition();
    inputs.armVelocity = encoder.getVelocity();
    inputs.armAmps = motor.getOutputCurrent();
  }

@Override
public void setEncoder(double value) {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'setEncoder'");
}
}
