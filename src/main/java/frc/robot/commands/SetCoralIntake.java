// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CoralShooter.CoralShooter;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class SetCoralIntake extends Command {
  private CoralShooter coralShooter;
  /** Creates a new SetCoralIntake. */
  public SetCoralIntake(CoralShooter coralShooter) {
    this.coralShooter = coralShooter;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(coralShooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    coralShooter.setDutyCycle(-0.3);
  }
  

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    coralShooter.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return coralShooter.getOuttakeSensor();
  }
}
