// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.CoralShooter;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.CommandLogger;
import org.littletonrobotics.junction.Logger;

public class CoralShooter extends SubsystemBase {
  private final CoralShooterIO io;
  public final CoralShooterIOInputsAutoLogged inputs = new CoralShooterIOInputsAutoLogged();

    /** Creates a new CoralOutake. */
    public CoralShooter(CoralShooterIO io) {
        this.io = io;
    }

    public void setDutyCycle(double speed) {
        io.setDutyCycle(speed);
    }

    public boolean getOuttakeSensor() {
        return inputs.outtakeSensor;
    }

    public void stop() {
        io.stop();
    }

    public Command runCmd(double dutyCycle) {
        return this.runEnd(() -> this.setDutyCycle(dutyCycle), () -> this.stop());
    }

    public Command waitUntilEmpty() {
        return Commands.waitUntil(() -> !this.getOuttakeSensor());
    }

    public Command waitUntilFull() {
        return Commands.waitUntil(() -> this.getOuttakeSensor());
    }

    public Command shootCmd() {
        String cmdName = "ShootCoral";
        return new InstantCommand(() -> CommandLogger.logCommandStart(cmdName))
            .andThen(waitUntilEmpty().raceWith(runCmd(-0.5)))
            .andThen(() -> CommandLogger.logCommandEnd(cmdName));
    }

    public Command intakeCmd() {
        String cmdName = "IntakeCoral";
        return new InstantCommand(() -> CommandLogger.logCommandStart(cmdName))
            .andThen(waitUntilFull().raceWith(runCmd(-0.25)))
            .andThen(() -> CommandLogger.logCommandEnd(cmdName));
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        io.updateInputs(inputs);
        Logger.processInputs("Coral Outtake", inputs);
    }
}
