// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.CoralIntake;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.CoralIntake.CoralIntakeIO.CoralIntakeIOInputs;

public class CoralIntake extends SubsystemBase {

    private CoralIntakeIO io;
    private CoralIntakeIOInputs inputs;
    /** Creates a new CoralIntake. */
    public CoralIntake(CoralIntakeIO io) {
        this.io = io;
    }

    public void setDutyCycle(double dutyCycle) {
        io.setDutyCycle(dutyCycle);
    }

    public void stop() {
        io.stop();
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
    }
}
