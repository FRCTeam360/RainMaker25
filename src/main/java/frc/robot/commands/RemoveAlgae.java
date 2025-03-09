// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.AlgaeArm.AlgaeArm;
import frc.robot.subsystems.AlgaeShooter.AlgaeShooter;
import frc.robot.subsystems.AlgaeTilt.AlgaeTilt;
import frc.robot.subsystems.Elevator.Elevator;
import frc.robot.subsystems.CoralShooter.CoralShooter;
import frc.robot.Constants.*;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class RemoveAlgae extends Command {
    private AlgaeArm algaeArm;
    private AlgaeShooter algaeShooter;
    private AlgaeTilt algaeTilt;
    private CoralShooter coralShooter;
    private Elevator elevator;

    private int level;
    private double height = SetPointConstants.ElevatorHeights.TELE_LEVEL_FOUR;

    /** Creates a new RemoveAlgae. */
    public RemoveAlgae(int level, AlgaeArm algaeArm, AlgaeShooter algaeShooter, AlgaeTilt algaeTilt,
            CoralShooter coralShooter,
            Elevator elevator) {
        this.algaeArm = algaeArm;
        this.algaeShooter = algaeShooter;
        this.algaeTilt = algaeTilt;
        this.coralShooter = coralShooter;
        this.elevator = elevator;
        this.level = level;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(algaeArm, algaeShooter, algaeTilt, elevator);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {}

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if (level == 3) {
            height = SetPointConstants.ElevatorHeights.TELE_LEVEL_FOUR - 10.0;
        } else if (level == 2) {
            height = SetPointConstants.ElevatorHeights.TELE_LEVEL_THREE - 5.5;
        }

        coralShooter.setDutyCycle(-1.0);
        algaeArm.setPosition(110.0);
        algaeTilt.setPosition(0.0);
        algaeShooter.setDutyCycle(-0.8);

        if (coralShooter.getVelocity() < -6000.0) {
            elevator.setElevatorPostion(height);
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        coralShooter.stop();
        algaeShooter.stop();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
