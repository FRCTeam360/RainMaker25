// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CoralShooter.CoralShooter;
import frc.robot.utils.CommandLogger;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class SmartIntake extends Command {
    private CoralShooter coralShooter;
    private Timer timer = new Timer();
    private Timer stallTimer = new Timer();
    private Timer unJammedTimer = new Timer();

    private boolean isFinised;

    private enum IntakeStates {
        EMPTY, FULL, JUST_INTAKE, JUST_OUTTAKE, JAMMED
    }

    private IntakeStates intakeStates;

    /** Creates a new SmartIntake. */
    private SmartIntake(CoralShooter coralShooter) {
        this.coralShooter = coralShooter;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(coralShooter);
        Logger.recordOutput("intake state", intakeStates);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        updateStates();
        isFinised = false;
        timer.reset();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        Logger.recordOutput("intake state", intakeStates);

        switch (intakeStates) {
            case JAMMED:
                stallTimer.reset();
                stallTimer.stop();
                coralShooter.setDutyCycle(0.2);
                unJammedTimer.start();
                if(unJammedTimer.hasElapsed(0.05)){
                    unJammedTimer.reset();
                    unJammedTimer.stop();
                    updateStates();
                }
                break;
            case EMPTY:
                stallTimer.start();
                coralShooter.setDutyCycle(-0.6);
                timer.reset();
                timer.stop();
                updateStates();
                break;
            case JUST_INTAKE:
                stallTimer.start();
                coralShooter.setDutyCycle(-0.1);
                timer.reset();
                timer.stop();
                updateStates();
                break;
            case JUST_OUTTAKE:
                stallTimer.start();
                coralShooter.setDutyCycle(0.1);
                timer.reset();
                timer.stop();
                updateStates();
                break;
            case FULL:
            default:
                coralShooter.stop();
                stallTimer.start();
                timer.start();
                if (timer.get() > 0.05) {
                    if (coralShooter.getOuttakeSensor() && coralShooter.getIntakeSensor()) {
                        timer.stop();
                        coralShooter.stop();
                        isFinised = true;
                    } else {
                        updateStates();
                    }
                }
                break;
        }

    }

    public void updateStates() {
        if(stallTimer.hasElapsed(.15) && coralShooter.getStatorCurrent() > 15.0 && Math.abs(coralShooter.getVelocity()) < .1){
            intakeStates = intakeStates.JAMMED;
        }else if(coralShooter.getIntakeSensor() && coralShooter.getOuttakeSensor()) {
            intakeStates = intakeStates.FULL;
        } else if (coralShooter.getIntakeSensor() && !coralShooter.getOuttakeSensor()) {
            intakeStates = intakeStates.JUST_INTAKE;
        } else if (!coralShooter.getIntakeSensor() && coralShooter.getOuttakeSensor()) {
            intakeStates = intakeStates.JUST_OUTTAKE;
        } else if (!coralShooter.getIntakeSensor() && !coralShooter.getOuttakeSensor()) {
            intakeStates = intakeStates.EMPTY;
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        coralShooter.stop();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return isFinised;
    }

    public static Command newCommand(CoralShooter coralShooter){
        return CommandLogger.logCommand(new SmartIntake(coralShooter), "SmartIntake");
    }
}
