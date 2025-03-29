// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.CoralShooter;

import edu.wpi.first.hal.HALUtil;
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

    private final Timer goodTimer = new Timer();

    /** Creates a new CoralOutake. */
    public CoralShooter(CoralShooterIO io) {
        this.io = io;
    }

    public void setDutyCycle(double speed) {
        io.setDutyCycle(speed);
    }

    public double getVelocity() {
        return inputs.outtakeVelocity;
    }

    public boolean getOuttakeSensor() {
        return inputs.outtakeSensor;
    }

    public boolean getIntakeSensor() {
        return inputs.intakeSensor;
    }


    public void stop() {
        io.stop();
    }

    public double getStatorCurrent() {
        return inputs.outtakeStatorCurrent;
    }

    public Command setDutyCycleCmd(double dutyCycle) {
        return this.runEnd(() -> this.setDutyCycle(dutyCycle), () -> this.stop());
    }

    public Command waitUntilIntakeSensor() {
        return Commands.waitUntil(() -> inputs.intakeSensor);
    }

    public Command waitUntilOuttakeSensor() {
        return Commands.waitUntil(() -> inputs.outtakeSensor);
    }

    public Command waitUntilEmpty() {
        return Commands.waitUntil(() -> (!this.getOuttakeSensor() && !this.getIntakeSensor()));
    }

    public Command waitUntilFull() {
        return Commands.waitUntil(() -> (this.getOuttakeSensor() && this.getIntakeSensor()));
    }

    public Command waitUntilEither() {
        return Commands.waitUntil(() -> (this.getOuttakeSensor() || this.getIntakeSensor()));
    }

    public Command pullAlgae() {
        return setDutyCycleCmd(-1.0);
    }

    public Command basicShootCmd() {
        String cmdName = "ShootCoral";
        return CommandLogger.logCommand(waitUntilEmpty().raceWith(setDutyCycleCmd(-0.35)), cmdName);
    }

    public Command basicIntakeCmd() {
        String cmdName = "IntakeCoral";
        return CommandLogger.logCommand(waitUntilEither().raceWith(setDutyCycleCmd(-0.35)), cmdName);
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
        return new ConditionalCommand(setDutyCycleCmd(1.0), setDutyCycleCmd(-0.4), () -> isUnjamming())
                .repeatedly()
                .alongWith(
                        new InstantCommand(
                                () -> {
                                    // testStall = true;
                                }));
    }

    public Command antiStallIntakeCmd() {
        String cmdName = "IntakeCoralEvenBetter";
        return CommandLogger.logCommand(
                waitUntilOuttakeSensor()
                        .deadlineFor(repeatOnStall()),
                cmdName);
    }

    public Command sensorIntakeCmd() {
        String cmdName = "IntakeCoral2";
        return CommandLogger.logCommand(
                waitUntilOuttakeSensor().deadlineFor(waitUntilIntakeSensor()
                        .deadlineFor(setDutyCycleCmd(-0.75))
                        .andThen(setDutyCycleCmd(-0.20))),
                cmdName);

        // return this.runEnd(() -> {
        //     if (!getIntakeSensor() && !getOuttakeSensor()) {
        //         setDutyCycle(-0.75);
        //     } else if (!getIntakeSensor()) {
        //         setDutyCycle(-0.2);
        //     } else if (getIntakeSensor() && getOuttakeSensor()) {
        //         goodTimer.start();

        //         if (goodTimer.get() == 0.1) {
        //             if (getIntakeSensor() )
        //         }
        //     }
        // });

    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        long periodicStartTime = HALUtil.getFPGATime();
        io.updateInputs(inputs);
        Logger.processInputs("Coral Outtake", inputs);
        long periodicLoopTime = HALUtil.getFPGATime() - periodicStartTime;
        Logger.recordOutput( "Coral Outtake: periodic loop time", (periodicLoopTime / 1000));
    }
}
