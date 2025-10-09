// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.AlgaeShooter;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.EncoderConfig;

import frc.robot.Constants;

public class AlgaeShooterIOPB extends AlgaeShooterIOCB {

 
  /** Creates a new AlgaeShooterIOWB. */
  public AlgaeShooterIOPB() {
    super(
        new SparkFlex(Constants.PracticeBotConstants.ALGAE_SHOOTER_FRONT_ID, MotorType.kBrushless),
        new SparkFlex(Constants.PracticeBotConstants.ALGAE_SHOOTER_BACK_ID, MotorType.kBrushless)
    );

    // TODO: add values
    final double kP = 0.0;
    final double kI = 0.0;
    final double kD = 0.0;
    
    ClosedLoopConfig closedLoopConfig = new ClosedLoopConfig();
    closedLoopConfig.pid(kP, kI, kD);
    frontConfig.apply(closedLoopConfig);
    backConfig.apply(closedLoopConfig);
    EncoderConfig encoderConfig = new EncoderConfig();
    encoderConfig.positionConversionFactor(positionConversionFactor);
    frontConfig.apply(encoderConfig);
    backConfig.apply(encoderConfig);
    backConfig.follow(Constants.PracticeBotConstants.ALGAE_SHOOTER_FRONT_ID, true);
    backConfig.inverted(true);
    frontConfig.inverted(false);
    
    algaeShooterMotorFront.configure(frontConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    algaeShooterMotorBack.configure(backConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }
}
