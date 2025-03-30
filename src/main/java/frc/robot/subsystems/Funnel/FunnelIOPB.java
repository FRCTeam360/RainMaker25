// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Funnel;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.EncoderConfig;
import com.revrobotics.spark.config.SparkFlexConfig;
import frc.robot.Constants;

public class FunnelIOPB implements FunnelIO {
    private final SparkFlex funnelMotor = new SparkFlex(Constants.PracticeBotConstants.FUNNEL_ID, MotorType.kBrushless);
    private final double positionConversionFactor = 1.0;

    private SparkFlexConfig motorConfig = new SparkFlexConfig();

    /** Creates a new AlgaeShooterIOWB. */
    public FunnelIOPB() {
        
        EncoderConfig encoderConfig = new EncoderConfig();
        encoderConfig.positionConversionFactor(positionConversionFactor);
        motorConfig.apply(encoderConfig);
        
        motorConfig.inverted(true);

        funnelMotor.configure(motorConfig, ResetMode.kResetSafeParameters,PersistMode.kPersistParameters
        );
    }

    public void updateInputs(FunnelIOInputs inputs) {
        inputs.funnelCurrent = funnelMotor.getOutputCurrent();
        inputs.funnelVoltage = funnelMotor.getBusVoltage();
        inputs.funnelVelocity = funnelMotor.getEncoder().getVelocity();
    }

    public void setDutyCycle(double dutyCycle) {
        funnelMotor.set(dutyCycle);
    }

    public void stop() {
        funnelMotor.stopMotor();
    }
}
