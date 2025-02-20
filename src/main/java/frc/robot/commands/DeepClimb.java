// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AlgaeShooter.AlgaeShooter;
import frc.robot.subsystems.ClimberWheel.ClimberWheel;
import frc.robot.subsystems.ClimberWinch.ClimberWinch;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class DeepClimb extends Command {
  private ClimberWinch winch;
  private ClimberWheel wheel;
  private AlgaeShooter shooter;

  /** Creates a new DeepClimb. */
  public DeepClimb(ClimberWinch winch, ClimberWheel wheel, AlgaeShooter shooter) {
    this.winch = winch;
    this.wheel = wheel;
    this.shooter = shooter;

    addRequirements(winch, wheel, shooter);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // algae shooter goes back to intake position
    shooter.setPositionCmd(0 /* placeholder */);
  
    // climber (winch) goes out
    winch.setPositionCmd(0 /* placeholder */);

    // interacts with cage

    // winch goes back while wheel runs
    winch.setPositionCmd(0 /* placeholder */);
    wheel.setDutyCycleCmd(0 /* placeholder */);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // winch is about 0 (winch is all the way back)
    if(winch.getPosition() < 0 /* placeholder */) {
      return true;
    }
    return false;
  }
}
