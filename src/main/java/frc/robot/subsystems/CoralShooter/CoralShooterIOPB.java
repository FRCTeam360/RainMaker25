// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.CoralShooter;

import com.reduxrobotics.sensors.canandcolor.Canandcolor;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import frc.robot.Constants;

/** Add your docs here. */
public class CoralShooterIOPB extends CoralShooterIOCB {

 
    public CoralShooterIOPB() {
        super();
        outtakeMotor = new SparkMax(Constants.PracticeBotConstants.CORAL_SHOOTER_ID, MotorType.kBrushless);
        intakeSensor = new Canandcolor(Constants.PracticeBotConstants.INTAKE_SENSOR_ID);
        outtakeSensor = new Canandcolor(Constants.PracticeBotConstants.OUTTAKE_SENSOR_ID);    

        sparkMaxConfig.idleMode(IdleMode.kBrake);
        sparkMaxConfig.inverted(false);
        outtakeMotor.configure(sparkMaxConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    private boolean isInIntakeSensor() {
        return intakeSensor.getProximity() < 0.06;
    }
}
