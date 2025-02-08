// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.subsystems.CommandSwerveDrivetrain;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class SysIdBLANKDirection extends Command {
    private CommandSwerveDrivetrain driveTrain;
    private CommandXboxController driverCont = new CommandXboxController(0);
   
    
  /** Creates a new SysIdBLANKDirection. */
  public SysIdBLANKDirection(CommandSwerveDrivetrain driveTrain) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveTrain);
    this.driveTrain = driveTrain;
   
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(driverCont.a().equals(true)) {
      driveTrain.sysIdDynamic(SysIdRoutine.Direction.kReverse);
    } else if(driverCont.x().equals(true)){
      driveTrain.sysIdDynamic(SysIdRoutine.Direction.kForward);
    } else if(driverCont.y().equals(true)){
      driveTrain.sysIdQuasistatic(SysIdRoutine.Direction.kForward);
    } else if(driverCont.b().equals(true)){
      driveTrain.sysIdQuasistatic(SysIdRoutine.Direction.kReverse);
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
