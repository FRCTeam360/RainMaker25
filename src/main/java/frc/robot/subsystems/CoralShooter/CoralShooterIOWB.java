// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.CoralShooter;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.DigitalInput;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import frc.robot.Constants;
import frc.robot.Constants.WoodbotConstants;
import frc.robot.subsystems.CoralShooter.CoralShooterIO.CoralShooterIOInputs;

/** Add your docs here. */
public class CoralShooterIOWB implements CoralShooterIO {

    private final SparkMax outtakeMotor = new SparkMax(WoodbotConstants.CORAL_SHOOTER_ID, MotorType.kBrushless);
    private final RelativeEncoder encoder = outtakeMotor.getEncoder();
    private final SparkMaxConfig sparkMaxConfig = new SparkMaxConfig();

    private final DigitalInput outtakeSensor = new DigitalInput(WoodbotConstants.OUTTAKE_SENSOR);
  
    private final double KP = 0.0;
    private final double KI = 0.0;
    private final double KD = 0.0;
    private final double KF = 0.0;

    public CoralShooterIOWB() {
        sparkMaxConfig.idleMode(IdleMode.kBrake);
        sparkMaxConfig.inverted(false);
        outtakeMotor.configure(sparkMaxConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void setDutyCycle(double dutyCycle) {
        outtakeMotor.set(dutyCycle);
    }

    private boolean getOuttakeSensor() {
        return !outtakeSensor.get();
    }

    public void stop() {
        outtakeMotor.stopMotor();
    }

    public void updateInputs(CoralShooterIOInputs inputs) {
        inputs.outtakeStatorCurrent = outtakeMotor.getOutputCurrent();
        inputs.outtakePosition = encoder.getPosition();
        inputs.outtakeVelocity = encoder.getVelocity();
        inputs.outtakeVoltage = outtakeMotor.getAppliedOutput() * outtakeMotor.getBusVoltage();
        inputs.shooterSensor = this.getOuttakeSensor();
    }
}
