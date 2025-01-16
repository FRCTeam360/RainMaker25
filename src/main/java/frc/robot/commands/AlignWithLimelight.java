// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Vision.Vision;
import java.util.function.DoubleSupplier;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AlignWithLimelight extends Command {
    private Vision vision;
    private CommandSwerveDrivetrain driveTrain;
    private double goalTY;
    private double maxSpeed;
    private double goalAngle;
    private double maxAngularRate;

    /** Creates a new AlignWithLimelight. */
    public AlignWithLimelight(
        Vision vision,
        CommandSwerveDrivetrain driveTrain,
        double goalAngle,
        double goalTY,
        double maxSpeed,
        double maxAngularRate
    ) {
        this.vision = vision;
        this.driveTrain = driveTrain;
        this.goalTY = goalTY;
        this.maxSpeed = maxSpeed;
        this.maxAngularRate = maxAngularRate;
        this.goalAngle = goalAngle;

        addRequirements(vision, driveTrain);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {}

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
      
        // driveTrain.driveFieldCentricFacingAngle(0.0, 0.0, goalAngle, 0); //changes angle to be square with apriltag
        System.out.println("running facing angle");
        if (Math.abs(driveTrain.getAngle() - goalAngle) < 5.0) {
            System.out.println("robot centric");
            driveTrain.robotCentricDrive(
                driveTrain.translationController.calculate(vision.getTXAdjusted()),
                0.0,
                0.0,
                maxSpeed,
                maxAngularRate
            ); //drives to apirltag
        }
    }


    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {}

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return Math.abs(vision.getTXAdjusted()) <= 0.0;
    }
}
