// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.CoralIntake.CoralIntake;
import frc.robot.subsystems.CoralShooter.CoralShooter;
import frc.robot.subsystems.Elevator.Elevator;
import edu.wpi.first.math.util.Units;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AdjustToScoreReef extends Command {
  private CommandSwerveDrivetrain drivetrain;
  private Translation2d initialTranslation;
  private Translation2d currentTranslation;
  private double maxSpeed;
  private double maxAngularRate;
  /** Creates a new ScoreInReef. */
  public AdjustToScoreReef(
    CommandSwerveDrivetrain drivetrain,
    double maxSpeed,
    double maxAngularRate) 
    { 
    this.drivetrain = drivetrain;
    this.maxSpeed = maxSpeed;
    this.maxAngularRate = maxAngularRate;
    
    addRequirements(drivetrain);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    initialTranslation = drivetrain.getPose().getTranslation();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    currentTranslation = drivetrain.getPose().getTranslation();
    double distance = Math.abs(currentTranslation.getDistance(initialTranslation));
    drivetrain.robotCentricDrive(0.1 * maxSpeed, 0.0, 0.0);
    if (distance > Units.inchesToMeters(6)) {
      drivetrain.robotCentricDrive(0.0, 0.0, 0.0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
