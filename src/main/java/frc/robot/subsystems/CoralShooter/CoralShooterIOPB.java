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
public class CoralShooterIOPB implements CoralShooterIO {

    private final SparkMax outtakeMotor = new SparkMax(Constants.PracticeBotConstants.CORAL_SHOOTER_ID, MotorType.kBrushless);
    private final RelativeEncoder encoder = outtakeMotor.getEncoder();
    private final SparkMaxConfig sparkMaxConfig = new SparkMaxConfig();

    private final Canandcolor intakeSensor = new Canandcolor(Constants.PracticeBotConstants.INTAKE_SENSOR_ID);
    private final Canandcolor outtakeSensor = new Canandcolor(Constants.PracticeBotConstants.OUTTAKE_SENSOR_ID);
  
    private final double KP = 0.0;
    private final double KI = 0.0;
    private final double KD = 0.0;
    private final double KF = 0.0;

    public CoralShooterIOPB() {
        sparkMaxConfig.idleMode(IdleMode.kBrake);
        sparkMaxConfig.inverted(false);
        outtakeMotor.configure(sparkMaxConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void setDutyCycle(double dutyCycle) {
        outtakeMotor.set(dutyCycle);
    }

    private boolean getOuttakeSensor() {
        return outtakeSensor.getProximity() < 5.0;
    }

    private boolean getIntakeSensor() {
        return intakeSensor.getProximity() < 5.0;
    }

    public void stop() {
        outtakeMotor.stopMotor();
    }

    public void updateInputs(CoralShooterIOInputs inputs) {
        inputs.outtakeStatorCurrent = outtakeMotor.getOutputCurrent();
        inputs.outtakePosition = encoder.getPosition();
        inputs.outtakeVelocity = encoder.getVelocity();
        inputs.outtakeVoltage = outtakeMotor.getAppliedOutput() * outtakeMotor.getBusVoltage();
        inputs.outtakeSensor = this.getOuttakeSensor();
        inputs.outtakeSensorProximity = outtakeSensor.getProximity();
        inputs.intakeSensor = this.getIntakeSensor();
        inputs.intakeSensorProximity = intakeSensor.getProximity();
    }
}
