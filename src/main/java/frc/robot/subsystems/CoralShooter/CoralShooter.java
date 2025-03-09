// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.CoralShooter;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.CommandLogger;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

public class CoralShooter extends SubsystemBase {
    private final CoralShooterIO io;
    public final CoralShooterIOInputsAutoLogged inputs = new CoralShooterIOInputsAutoLogged();

    private DoubleSupplier elevatorHeight;

    private final Timer stallTimer = new Timer();
    private final Timer unjamTimer = new Timer();
    private boolean testStall = true;

    private final Color8Bit color = new Color8Bit(Color.kCoral);
    private final LoggedMechanism2d mech2d = new LoggedMechanism2d(30, 50, new Color8Bit(Color.kBlue));
    private final LoggedMechanismRoot2d mech2dRoot = mech2d.getRoot("shooter root", 10, 0);
    private final LoggedMechanismLigament2d mech2dSide1 = mech2dRoot.append(new LoggedMechanismLigament2d("side1", 10, 340, 5, color));
    private final LoggedMechanismLigament2d mech2dSide2 = mech2dRoot.append(new LoggedMechanismLigament2d("side2", 5, 270, 5, color));
    private final LoggedMechanismLigament2d mech2dSide3 = mech2dSide1.append(new LoggedMechanismLigament2d("side3", 5, 290, 5, color));
    private final LoggedMechanismLigament2d mech2dSide4 = mech2dSide2.append(new LoggedMechanismLigament2d("side4", 10, 70, 5, color));

    /** Creates a new CoralOutake. */
    public CoralShooter(CoralShooterIO io, DoubleSupplier elevatorHeight) {
        this.io = io;
        this.elevatorHeight = elevatorHeight;
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

    public Command waitUntilEmpty() {
        return Commands.waitUntil(() -> (!this.getOuttakeSensor() && !this.getIntakeSensor()));
    }

    public Command waitUntilOuttakeSensor() {
        return Commands.waitUntil(() -> this.getOuttakeSensor());
    }

    public Command waitUntilFull() {
        return Commands.waitUntil(() -> (this.getOuttakeSensor() || this.getIntakeSensor()));

    }

    public Command waitUntilIntakeSensor() {
        return Commands.waitUntil(() -> inputs.intakeSensor);
    }

    public Command pullAlgae() {
        return setDutyCycleCmd(-1.0);
    }

    public Command basicShootCmd() {
        String cmdName = "ShootCoral";
        return CommandLogger.logCommand(waitUntilEmpty().raceWith(setDutyCycleCmd(-0.40)), cmdName);
    }

    public Command basicIntakeCmd() {
        String cmdName = "IntakeCoral";
        return CommandLogger.logCommand(waitUntilFull().raceWith(setDutyCycleCmd(-0.35)), cmdName);
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
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        mech2dRoot.setPosition(10, elevatorHeight.getAsDouble());
        io.updateInputs(inputs);
        Logger.recordOutput("Coral Mech", mech2d);
        Logger.processInputs("Coral Outtake", inputs);
    }
}
