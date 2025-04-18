// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.CoralShooter;

import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.signals.UpdateModeValue;
import com.reduxrobotics.sensors.canandcolor.Canandcolor;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import frc.robot.Constants;

/** Add your docs here. */
public class CoralShooterIOCB implements CoralShooterIO {
    private final SparkMax outtakeMotor = new SparkMax(
        Constants.CompBotConstants.CORAL_SHOOTER_ID,
        MotorType.kBrushless
    );
    
    private final RelativeEncoder encoder = outtakeMotor.getEncoder();
    private final SparkMaxConfig sparkMaxConfig = new SparkMaxConfig();
    CANrangeConfiguration canRangeConfig = new CANrangeConfiguration();

    // private final Canandcolor intakeSensor = new Canandcolor(Constants.CompBotConstants.INTAKE_SENSOR_ID);
    // private final Canandcolor outtakeSensor = new Canandcolor(Constants.CompBotConstants.OUTTAKE_SENSOR_ID);

    private final CANrange intakeSensor = new CANrange(
        Constants.CompBotConstants.INTAKE_SENSOR_ID,
        Constants.CompBotConstants.CANBUS_NAME
    );
    private final CANrange outtakeSensor = new CANrange(
        Constants.CompBotConstants.OUTTAKE_SENSOR_ID,
        Constants.CompBotConstants.CANBUS_NAME
    );

    private final double KP = 0.0;
    private final double KI = 0.0;
    private final double KD = 0.0;
    private final double KF = 0.0;

    public CoralShooterIOCB() {
        sparkMaxConfig.idleMode(IdleMode.kBrake);
        sparkMaxConfig.inverted(false);
        sparkMaxConfig.smartCurrentLimit(25, 5);

        outtakeMotor.configure(
            sparkMaxConfig,
            ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters
        );

        canRangeConfig.ProximityParams.MinSignalStrengthForValidMeasurement = 2000; // If CANrange has a signal strength of at least 2000, it is a valid measurement.
        canRangeConfig.ProximityParams.ProximityThreshold = 0.05; // If CANrange detects an object within 0.1 meters, it will trigger the "isDetected" signal.

        // canRangeConfig.ToFParams.UpdateMode = UpdateModeValue.ShortRange100Hz; // Make the CANrange update as fast as possible at 100 Hz. This requires short-range mode.

        intakeSensor.getConfigurator().apply(canRangeConfig);
        outtakeSensor.getConfigurator().apply(canRangeConfig);
    }

    public void setDutyCycle(double dutyCycle) {
        outtakeMotor.set(dutyCycle);
    }

    private boolean isInIntakeSensor() {
        // return intakeSensor.getProximity() < 0.1;
        return intakeSensor.getIsDetected().getValue();
    }

    private boolean isInOuttakeSensor() {
        // return outtakeSensor.getProximity() < 0.1;
        return outtakeSensor.getIsDetected().getValue();
    }

    public void stop() {
        outtakeMotor.stopMotor();
    }

    public void updateInputs(CoralShooterIOInputs inputs) {
        inputs.outtakeStatorCurrent = outtakeMotor.getOutputCurrent();
        inputs.outtakePosition = encoder.getPosition();
        inputs.outtakeVelocity = encoder.getVelocity();
        inputs.outtakeVoltage = outtakeMotor.getAppliedOutput() * outtakeMotor.getBusVoltage();
        inputs.outtakeSensor = this.isInOuttakeSensor();
        // inputs.outtakeSensorProximity = outtakeSensor.getProximity();
        inputs.outtakeSensorProximity = outtakeSensor.getDistance().getValueAsDouble();
        inputs.intakeSensor = this.isInIntakeSensor();
        // inputs.intakeSensorProximity = intakeSensor.getProximity();
        inputs.intakeSensorProximity = intakeSensor.getDistance().getValueAsDouble();
    }
}
