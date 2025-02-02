// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.XboxController;
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
    private double angleToFace = 0.0;
    private XboxController driverCont;
    private double lastTXValue;
    private double lastTYValue;

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
        if (driveTrain.getAngle() >= -30.0 || driveTrain.getAngle() <= 30.0) {
            angleToFace = 0.0;
        } else if (driveTrain.getAngle() <= -30.0 || driveTrain.getAngle() >= -90.0) {
            angleToFace = -60.0;
        } else if (driveTrain.getAngle() <= -90.0 || driveTrain.getAngle() >= -150.0) {
            angleToFace = -120.0;
        } else if (driveTrain.getAngle() <= -150.0 || driveTrain.getAngle() >= 150.0) {
            angleToFace = 180.0;
        } else if (driveTrain.getAngle() <= 150.0 || driveTrain.getAngle() >= 90.0) {
            angleToFace = 120.0;
        } else if (driveTrain.getAngle() >= 30.0 || driveTrain.getAngle() <= 90.0) {
            angleToFace = 60.0;
        }
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        lastTXValue = vision.getTXRaw();
        lastTYValue = vision.getTYRaw();

     if(vision.getTV() == 1) {
            driveTrain.driveFieldCentricFacingAngle(
                driveTrain.translationController.calculate(vision.getTYRaw(), goalTY), //forward & backward motion
                   -driveTrain.translationController.calculate(vision.getTXRaw(), goalTX),
                    angleToFace, //side to side motion
                    maxSpeed
                );
        } else {
           driveTrain.driveFieldCentricFacingAngle(0.0, 0.0, angleToFace, maxSpeed);
       withTimeout(1);
        }
       

    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {}

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        // return false;
        return Math.abs( vision.getTXRaw() - goalTX) < 2.0 &&  Math.abs(vision.getTYRaw()  - goalTY) < 0.5;
    }
}
