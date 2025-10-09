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

    protected SparkMax outtakeMotor;
    protected RelativeEncoder encoder;
    protected SparkMaxConfig sparkMaxConfig = new SparkMaxConfig();

    protected Canandcolor intakeSensor;
    protected Canandcolor outtakeSensor;

    protected final double KP = 0.0;
    protected final double KI = 0.0;
    protected final double KD = 0.0;
    protected final double KF = 0.0;

    public CoralShooterIOCB() {
        outtakeMotor = new SparkMax(Constants.CompBotConstants.CORAL_SHOOTER_ID, MotorType.kBrushless);
        encoder = outtakeMotor.getEncoder();
        
        intakeSensor = new Canandcolor(Constants.CompBotConstants.INTAKE_SENSOR_ID);
        outtakeSensor = new Canandcolor(Constants.CompBotConstants.OUTTAKE_SENSOR_ID);
  
        sparkMaxConfig.idleMode(IdleMode.kBrake);
        sparkMaxConfig.inverted(false);
        outtakeMotor.configure(sparkMaxConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void setDutyCycle(double dutyCycle) {
        outtakeMotor.set(dutyCycle);
    }

    protected boolean isInOuttakeSensor() {
        return outtakeSensor.getProximity() < 0.1;
    }

    private boolean isInIntakeSensor() {
        return intakeSensor.getProximity() < 0.0575;
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
