// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AlgaeRoller.AlgaeRoller;
import frc.robot.subsystems.AlgaeShooter.AlgaeShooter;
import frc.robot.subsystems.AlgaeTilt.AlgaeTilt;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ShuffleboardTuner extends Command {
    private AlgaeTilt algaeTilt;
    private AlgaeShooter algaeShooter;
    private AlgaeRoller algaeRoller;

    private double tiltAngle = 0.0;
    private double shooterVelocity = 0.0;
    private double rollerDutyCycle = 0.0;

    private double newTiltAngle = tiltAngle;
    private double newShooterVelocity = shooterVelocity;
    private double newRollerDutyCycle = rollerDutyCycle;
    
  /** Creates a new ShuffleboardTuner. */
  public ShuffleboardTuner(AlgaeTilt algaeTilt, AlgaeShooter algaeShooter, AlgaeRoller algaeRoller) {
    this.algaeTilt = algaeTilt;
    this.algaeShooter = algaeShooter;
    this.algaeRoller = algaeRoller;
    
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(algaeTilt, algaeShooter, algaeRoller);

    SmartDashboard.putNumber("tilt angle", tiltAngle);
    SmartDashboard.putNumber("shooter velocity", shooterVelocity);
    SmartDashboard.putNumber("roller velocity", rollerDutyCycle);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    newTiltAngle = SmartDashboard.getNumber("tilt angle", tiltAngle);
    newShooterVelocity = SmartDashboard.getNumber("tilt angle", shooterVelocity);
    newRollerDutyCycle = SmartDashboard.getNumber("roller dutycycle", rollerDutyCycle);

    if (tiltAngle != newTiltAngle) {
        tiltAngle = newTiltAngle;
    }

    if (tiltAngle != newShooterVelocity) {
        shooterVelocity = newShooterVelocity;
    }

    if (tiltAngle != newRollerDutyCycle) {
        rollerDutyCycle = newRollerDutyCycle;
    }

    algaeTilt.setPosition(tiltAngle);
    algaeShooter.setVelocity(shooterVelocity);
    algaeRoller.setDutyCycle(rollerDutyCycle);

    SmartDashboard.putNumber("tilt angle", tiltAngle);
    SmartDashboard.putNumber("shooter velocity", shooterVelocity);
    SmartDashboard.putNumber("roller velocity", rollerDutyCycle);
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
