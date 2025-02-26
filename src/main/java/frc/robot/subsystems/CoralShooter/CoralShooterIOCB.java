// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.CoralShooter;

import com.reduxrobotics.sensors.canandcolor.Canandcolor;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import frc.robot.Constants;

/** Add your docs here. */
public class CoralShooterIOCB implements CoralShooterIO {

    private final SparkMax outtakeMotor = new SparkMax(Constants.CompBotConstants.CORAL_SHOOTER_ID, MotorType.kBrushless);
    private final RelativeEncoder encoder = outtakeMotor.getEncoder();
    private final SparkMaxConfig sparkMaxConfig = new SparkMaxConfig();

    private final Canandcolor intakeSensor = new Canandcolor(Constants.CompBotConstants.INTAKE_SENSOR_ID);
    private final Canandcolor outtakeSensor = new Canandcolor(Constants.CompBotConstants.OUTTAKE_SENSOR_ID);
  
    private final double KP = 0.0;
    private final double KI = 0.0;
    private final double KD = 0.0;
    private final double KF = 0.0;

    public CoralShooterIOCB() {
        sparkMaxConfig.idleMode(IdleMode.kBrake);
        sparkMaxConfig.inverted(false);
        outtakeMotor.configure(sparkMaxConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void setDutyCycle(double dutyCycle) {
        outtakeMotor.set(dutyCycle);
    }

    private boolean isInOuttakeSensor() {
        return outtakeSensor.getProximity() < 0.1;
    }

    private boolean isInIntakeSensor() {
        return intakeSensor.getProximity() < 0.06;
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
        inputs.outtakeSensorProximity = outtakeSensor.getProximity();
        inputs.intakeSensor = this.isInIntakeSensor();
        inputs.intakeSensorProximity = intakeSensor.getProximity();
    }
}
