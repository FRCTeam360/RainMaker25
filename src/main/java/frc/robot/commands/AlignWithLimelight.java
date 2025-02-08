// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Vision.Vision;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AlignWithLimelight extends Command {
    private Vision vision;
    private CommandSwerveDrivetrain driveTrain;
    private double goalTY;

    /** Creates a new AlignWithLimelight. */
    public AlignWithLimelight(
        Vision vision,
        CommandSwerveDrivetrain driveTrain,
        double goalAngle,
        double goalTY
    ) {
        this.vision = vision;
        this.driveTrain = driveTrain;
        this.goalTY = goalTY;

        addRequirements(driveTrain);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {}

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        driveTrain.robotCentricDrive(
            driveTrain.translationController.calculate(vision.getTYRaw(), goalTY), //forward & backward motion
            driveTrain.translationController.calculate(vision.getTXRaw(), 0.0), //side to side motion
            0.0
        );
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {}

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return Math.abs(vision.getTXRaw()) <= 1.0 && Math.abs(vision.getTYRaw() - 3.0) <= 2.0;
    }
}
