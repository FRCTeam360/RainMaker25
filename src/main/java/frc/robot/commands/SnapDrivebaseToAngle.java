// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Vision.Vision;
import frc.robot.subsystems.Vision.VisionIOLimelight;
import frc.robot.utils.LimelightHelpers;

import java.util.Map;
import java.util.Optional;

import org.ejml.interfaces.decomposition.SingularValueDecomposition;
import org.littletonrobotics.junction.Logger;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class SnapDrivebaseToAngle extends Command {
    private CommandSwerveDrivetrain driveTrain;
    private CommandXboxController driverCont = new CommandXboxController(0);
    private double angleToFace = 0.0;
    private Rotation2d angleToFaceRotation2d;
    private Vision aprilTagID;
    private boolean endEarly;
    private int pipeline;

    private final String CMD_NAME = "SnapDrivebaseToAngle: ";

    private static final Map<Integer, Double> tagIDToAngle = Map.ofEntries(
            Map.entry(21, 180.0),
            Map.entry(7, 180.0),
            Map.entry(22, 120.0),
            Map.entry(6, 120.0),
            Map.entry(17, 60.0),
            Map.entry(11, 60.0),
            Map.entry(18, 0.0),
            Map.entry(10, 0.0),
            Map.entry(19, -60.0),
            Map.entry(9, -60.0),
            Map.entry(20, -120.0),
            Map.entry(8, -120.0));

    /** Creates a new SnapDrivebaseToAngle. */
    public SnapDrivebaseToAngle(Vision aprilTagID, CommandSwerveDrivetrain driveTrain, int pipeline) {
        this.aprilTagID = aprilTagID;
        this.driveTrain = driveTrain;
        this.pipeline = pipeline;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(driveTrain);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {

        aprilTagID.setPipeline(pipeline);

        double angleToFace = driveTrain.getAngle();
        int priorityID = aprilTagID.getAprilTagID();

        endEarly = true;
        if (aprilTagID.getTV() == 1 && tagIDToAngle.containsKey(priorityID)) {
            endEarly = false;
            angleToFace = tagIDToAngle.get(priorityID);
        }

        Optional<Alliance> alliance = DriverStation.getAlliance();
        if (alliance.isPresent() && alliance.get() == Alliance.Red) {
            if (angleToFace <= 0.0) {
                angleToFace = angleToFace + 180.0;
            } else {
                angleToFace = angleToFace - 180.0;
            }
        }

        angleToFaceRotation2d = Rotation2d.fromDegrees(angleToFace);

        Logger.recordOutput(CMD_NAME + "AngleToFace", angleToFace);

        LimelightHelpers.setPriorityTagID("limelight", priorityID);

    }

    public static Translation2d rotateTranslation(
        Translation2d translationToRotate,
        Rotation2d rotation
    ) {
        return translationToRotate.rotateBy(rotation);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if (endEarly) return;

        if (aprilTagID.getTV() == 1) {
            driveTrain.driveFieldCentricFacingAngle(
                Math.pow(MathUtil.applyDeadband(-driverCont.getLeftY(), 0.1), 2.0),
                Math.pow(MathUtil.applyDeadband(-driverCont.getLeftX(), 0.1), 2.0),
                angleToFace);
        } else {
            driveTrain.driveFieldCentricFacingAngle(0.0, 0.0, angleToFaceRotation2d.getDegrees());
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        LimelightHelpers.setPriorityTagID("limelight", -1);
        driveTrain.xOut();
    };

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        boolean onHeading = driveTrain.isAtRotationSetpoint();
        Logger.recordOutput(CMD_NAME + "onHeading", onHeading);
        return endEarly || driveTrain.isAtRotationSetpoint();
    }
}
