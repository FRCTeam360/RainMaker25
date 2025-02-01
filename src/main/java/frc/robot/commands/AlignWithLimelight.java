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

import javax.annotation.processing.SupportedOptions;

import org.littletonrobotics.junction.Logger;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AlignWithLimelight extends Command {
    private Vision vision;
    private CommandSwerveDrivetrain driveTrain;
    private double goalTY;
    private double goalTX;
    private double maxSpeed;
    private double maxAngularRate;

    /** Creates a new AlignWithLimelight. */
    public AlignWithLimelight(
        Vision vision,
        CommandSwerveDrivetrain driveTrain,
        double goalTY,
        double goalTX,
        double maxSpeed,
        double maxAngularRate
    ) {
        this.vision = vision;
        this.driveTrain = driveTrain;
        this.goalTY = goalTY;
        this.maxSpeed = maxSpeed;
        this.maxAngularRate = maxAngularRate;
        this.goalTX = goalTX;

        addRequirements(driveTrain);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        System.out.println("HUHEIUIUHIUHUISHEIFUHUSHIUE");
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        driveTrain.robotCentricDrive(
            driveTrain.translationController.calculate(vision.getTYRaw(), goalTY), //forward & backward motion
            driveTrain.translationController.calculate(vision.getTXRaw(), goalTX), //side to side motion
            0.0,
            maxSpeed,
            maxAngularRate
        );
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {}

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        System.out.println("HELLOHELLOEHHOEJOJFEJIHFIE");
        return false;
        // return Math.abs( (vision.getTXRaw()) - goalTX) > 0.5 &&  Math.abs(vision.getTYRaw()  - goalTY) > 0.5;
    }
}
