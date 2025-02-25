// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.utils.CommandLogger;
import frc.robot.utils.RobotUtils;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class FaceAngle extends Command {
  private final CommandSwerveDrivetrain drivetrain;
  private double desiredAngle;
  /** Creates a new FaceAngle. */
  private FaceAngle(CommandSwerveDrivetrain drivetrain, Rotation2d desiredRotation) {
    this.desiredAngle = desiredRotation.getDegrees();
    this.drivetrain = drivetrain;
    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    drivetrain.driveFieldCentricFacingAngle(0.0, 0.0, desiredAngle);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return drivetrain.isAtRotationSetpoint();
  }

  public static Command getCommand(CommandSwerveDrivetrain drivetrain, Rotation2d desiredRotation) {
    return CommandLogger.logCommand(Commands.either(
      // Red side, flipped
      new FaceAngle(drivetrain, RobotUtils.flipForRedAlliancePerspective(desiredRotation)),
      // Blue side, no flipping
      new FaceAngle(drivetrain, desiredRotation),
      () -> DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red
      ), "Face Angle");
  }
}
