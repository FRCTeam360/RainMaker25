// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.*;
import frc.robot.Constants.SetPointConstants.ElevatorHeights;
import frc.robot.subsystems.CoralShooter.CoralShooter;
import frc.robot.subsystems.Elevator.Elevator;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class HasCoral extends Command {
    private final CoralShooter coralShooter;
    private final Elevator elevator;
    private final String CMD_NAME = "HasCoral";

    private boolean isFinished;

  /** Creates a new HasCoral. */
  public HasCoral(CoralShooter coralShooter, Elevator elevator) {
    this.coralShooter = coralShooter;
    this.elevator = elevator;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(coralShooter, elevator);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    isFinished = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    if(coralShooter.getIntakeSensor() || coralShooter.getOuttakeSensor()) {

        new SequentialCommandGroup(
            elevator.setElevatorHeight(ElevatorHeights.AUTO_LEVEL_FOUR),
            coralShooter.basicShootCmd()
        );

        isFinished = true;

    } else {
        isFinished = true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    Logger.recordOutput(CMD_NAME, isFinished);
    return isFinished;
  }
}
