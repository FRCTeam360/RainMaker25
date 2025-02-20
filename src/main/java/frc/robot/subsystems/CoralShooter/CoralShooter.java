// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.CoralShooter;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.CommandLogger;
import org.littletonrobotics.junction.Logger;

public class CoralShooter extends SubsystemBase {
    private final CoralShooterIO io;
    public final CoralShooterIOInputsAutoLogged inputs = new CoralShooterIOInputsAutoLogged();

    private final Timer stallTimer = new Timer();
    private final Timer unjamTimer = new Timer();
    private boolean testStall = true;

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

    public Command waitUntilIntakeSensor() {
        return Commands.waitUntil(() -> inputs.intakeSensor);
    }

    public Command shootCmd() {
        String cmdName = "ShootCoral";
        return CommandLogger.logCommand(waitUntilEmpty().raceWith(runCmd(-0.6)), cmdName);
    }

    public Command intakeCmd() {
        String cmdName = "IntakeCoral";
        return CommandLogger.logCommand(waitUntilFull().raceWith(runCmd(-0.25)), cmdName);
    }

    private boolean isStalling() {
        boolean stalling = inputs.outtakeStatorCurrent > 15.0;
        // stalling = testStall;
        if (stalling && !stallTimer.isRunning()) {
            stallTimer.restart();
        } else if (stalling && stallTimer.hasElapsed(0.2)) {
            stallTimer.stop();
            return true;
        } else if (!stalling) {
            stallTimer.stop();
        }
        return false;
    }

    private boolean isUnjamming() {
        boolean isStalled = isStalling();
        if (isStalled) {
            // testStall = false;
            unjamTimer.restart();
            return true;
        } else if (unjamTimer.isRunning() && !unjamTimer.hasElapsed(0.2)) {
            return true;
        } else if (unjamTimer.hasElapsed(0.2)) {
            unjamTimer.stop();
        }
        return false;
    }

    private Command repeatOnStall() {
        return new ConditionalCommand(runCmd(1.0), runCmd(-0.3), () -> isUnjamming())
            .repeatedly()
            .alongWith(
                new InstantCommand(
                    () -> {
                        // testStall = true;
                    }
                )
            );
    }

    public Command antiStallIntakeCmd() {
        String cmdName = "IntakeCoralEvenBetter";
        return CommandLogger.logCommand(
            waitUntilIntakeSensor()
                .deadlineFor(repeatOnStall())
                .andThen(waitUntilFull().deadlineFor(runCmd(-0.2))),
            cmdName
        );
    }

    public Command betterIntakeCmd() {
        String cmdName = "IntakeCoral2";
        return CommandLogger.logCommand(
            waitUntilIntakeSensor()
                .deadlineFor(runCmd(-1.0))
                .andThen(waitUntilFull().deadlineFor(runCmd(-0.3))),
            cmdName
        );
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        io.updateInputs(inputs);
        Logger.processInputs("Coral Outtake", inputs);
    }
}
