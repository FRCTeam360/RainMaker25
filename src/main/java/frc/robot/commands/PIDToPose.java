// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.utils.CommandLogger;
import frc.robot.utils.RobotUtils;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class PIDToPose extends Command {
    private final CommandSwerveDrivetrain drivetrain;
    private final Pose2d setpointPose;
    private final String LOGGING_PREFIX = "PIDToPose: ";

    /** Creates a new FaceAngle. */
    private PIDToPose(CommandSwerveDrivetrain drivetrain, Pose2d setpointPose) {
        this.setpointPose = setpointPose;
        this.drivetrain = drivetrain;
        addRequirements(drivetrain);

        Logger.recordOutput(LOGGING_PREFIX + "isAtRotationSetpoint", false);
        Logger.recordOutput(LOGGING_PREFIX + "isAtPoseXSetPoint", false);
        Logger.recordOutput(LOGGING_PREFIX + "isAtPoseYSetPoint", false);

        Logger.recordOutput(LOGGING_PREFIX + "headingSetPoint", 0.0);
        Logger.recordOutput(LOGGING_PREFIX + "poseXSetpoint", 0.0);
        Logger.recordOutput(LOGGING_PREFIX + "poseYSetpoint", 0.0);

        Logger.recordOutput(LOGGING_PREFIX + "headingPositionError", 0.0);
        Logger.recordOutput(LOGGING_PREFIX + "poseXPositionError", 0.0);
        Logger.recordOutput(LOGGING_PREFIX + "poseYPositionError", 0.0);

        Logger.recordOutput(LOGGING_PREFIX + "headingVelocityError", 0.0);
        Logger.recordOutput(LOGGING_PREFIX + "poseXVelocityError", 0.0);
        Logger.recordOutput(LOGGING_PREFIX + "poseYVelocityError", 0.0);

    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        drivetrain.driveToPose(setpointPose);
        final double headingSetpoint = setpointPose.getRotation().getDegrees();
        final double poseXSetpoint = setpointPose.getX();
        final double poseYSetpoint = setpointPose.getY();

        Logger.recordOutput(LOGGING_PREFIX + "positionSetpoint", headingSetpoint);
        Logger.recordOutput(LOGGING_PREFIX + "headingSetpoint", headingSetpoint);
        Logger.recordOutput(LOGGING_PREFIX + "poseXSetpoint", poseXSetpoint);
        Logger.recordOutput(LOGGING_PREFIX + "poseYSetpoint", poseYSetpoint);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        drivetrain.driveToPose(setpointPose);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {}


    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        final boolean isAtHeadingSetpoint = drivetrain.isAtRotationSetpoint();
        final boolean isAtPoseXSetPoint = drivetrain.isAtPoseXSetpoint();
        final boolean isAtPoseYSetPoint = drivetrain.isAtPoseYSetpoint();

        final double headingPositionError = Math.toDegrees(drivetrain.getHeadingControllerPositionError());
        final double poseXPositionError = drivetrain.getPoseXControllerPositionError();
        final double poseYPositionError = drivetrain.getPoseYControllerPositionError();

        final double headingVelocityError = Math.toDegrees(drivetrain.getHeadingControllerVelocityError());
        final double poseXVelocityError = drivetrain.getPoseXControllerVelocityError();
        final double poseYVelocityError = drivetrain.getPoseYControllerVelocityError();

        Logger.recordOutput(LOGGING_PREFIX + "isAtRotationSetpoint", isAtHeadingSetpoint);
        Logger.recordOutput(LOGGING_PREFIX + "isAtPoseXSetPoint", isAtPoseXSetPoint);
        Logger.recordOutput(LOGGING_PREFIX + "isAtPoseYSetPoint", isAtPoseYSetPoint);

        Logger.recordOutput(LOGGING_PREFIX + "headingPositionError", headingPositionError);
        Logger.recordOutput(LOGGING_PREFIX + "poseXPositionError", poseXPositionError);
        Logger.recordOutput(LOGGING_PREFIX + "poseYPositionError", poseYPositionError);

        Logger.recordOutput(LOGGING_PREFIX + "headingVelocityError", headingVelocityError);
        Logger.recordOutput(LOGGING_PREFIX + "poseXVelocityError", poseXVelocityError);
        Logger.recordOutput(LOGGING_PREFIX + "poseYVelocityError", poseYVelocityError);

        return isAtHeadingSetpoint && isAtPoseXSetPoint && isAtPoseYSetPoint;
    }

    public static Command getCommand(CommandSwerveDrivetrain drivetrain, Pose2d setpointPose) {
        return CommandLogger.logCommand(
                new PIDToPose(drivetrain, setpointPose),
                "DriveToPose");
    }
}
