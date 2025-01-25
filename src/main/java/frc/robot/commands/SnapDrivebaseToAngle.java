// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Vision.Vision;

import java.util.Optional;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class SnapDrivebaseToAngle extends Command {
    private CommandSwerveDrivetrain driveTrain;
    private CommandXboxController driverCont = new CommandXboxController(0);
    private double maxSpeed;
    private double angleToFace = 0.0;
    private Vision vision;

    /** Creates a new SnapDrivebaseToAngle. */
    public SnapDrivebaseToAngle(CommandSwerveDrivetrain driveTrain, double maxSpeed, Vision vision) {
        this.driveTrain = driveTrain;
        this.maxSpeed = maxSpeed;
        this.vision = vision;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(driveTrain);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        if (vision.getAprilTagID() == 18 || vision.getAprilTagID() == 7) {
            angleToFace = 0.0;
        } else if (vision.getAprilTagID() == 19 || vision.getAprilTagID() == 6)  {
            angleToFace = -60.0;
        } else if (vision.getAprilTagID() == 20 || vision.getAprilTagID() == 11) {
            angleToFace = -120.0;
        } else if (vision.getAprilTagID() == 21 || vision.getAprilTagID() == 10) {
            angleToFace = 180.0;
        } else if (vision.getAprilTagID() == 22 || vision.getAprilTagID() == 9) {
            angleToFace = 120.0;
        } else if (vision.getAprilTagID() == 17 || vision.getAprilTagID() == 8) {
            angleToFace = 60.0;
        }
    }
    
    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        System.out.println(angleToFace);
        // driveTrain.driveFieldCentricFacingAngle(
        //     Math.pow(MathUtil.applyDeadband(-driverCont.getLeftY(), 0.1), 2.0),
        //     Math.pow(MathUtil.applyDeadband(-driverCont.getLeftX(), 0.1), 2.0),
        //     angleToFace,
        //     maxSpeed
        // );
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {}

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return Math.abs(driveTrain.getAngle() - angleToFace) <= 1.0;
    }
}
