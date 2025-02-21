// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.AlgaeTilt;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.EncoderConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class AlgaeTiltIOPB implements AlgaeTiltIO {
  private final SparkMax motor = new SparkMax(Constants.PracticeBotConstants.ALGAE_TILT, MotorType.kBrushless);
  private final AbsoluteEncoder encoder =  motor.getAbsoluteEncoder();

  private final double kP = 0.0;
  private final double kI = 0.0;
  private final double kD = 0.0;

  private final double positionConversionFactor = 1.0;
  private final SparkMaxConfig sparkMaxConfig = new SparkMaxConfig();

  /** Creates a new AlgaeIntakeIOPB. */
  public AlgaeTiltIOPB() {
    sparkMaxConfig.idleMode(IdleMode.kBrake);
    sparkMaxConfig.inverted(false);
    ClosedLoopConfig closedLoopConfig = new ClosedLoopConfig();
    closedLoopConfig.pid(kP, kI, kD);
    sparkMaxConfig.apply(closedLoopConfig);
    EncoderConfig encoderConfig = new EncoderConfig();
    encoderConfig.positionConversionFactor(positionConversionFactor);
    sparkMaxConfig.apply(encoderConfig);
    motor.configure(sparkMaxConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
}

  public void setDutyCycle(double dutyCycle) {
    motor.set(dutyCycle);
  }

  public void setPosition(double position) {
    motor.getClosedLoopController().setReference(position, ControlType.kPosition);
  }

  public void updateInputs(AlgaeTiltIOInputs inputs) {
    inputs.armDutyCycle = motor.get();
    inputs.armPosition = encoder.getPosition();
    inputs.armVelocity = encoder.getVelocity(); 
  }
}
