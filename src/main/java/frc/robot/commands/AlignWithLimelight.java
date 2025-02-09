// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Vision.Vision;
import frc.robot.utils.LimelightHelpers;
import java.util.Map;
import java.util.Objects;
import java.util.Optional;
import org.littletonrobotics.junction.Logger;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AlignWithLimelight extends Command {
    private Vision vision;
    private CommandSwerveDrivetrain driveTrain;
    private double goalTY;
    private double goalTX;
    private double lastTXValue;
    private double lastTYValue;
    private Rotation2d angleToFaceRotation2d;
    private double rotatedVelocityX;
    private double rotatedVelocityY;
    private boolean endEarly = false;
    private XboxController driverCont;
    private SlewRateLimiter forwardsAccelerationLimit = new SlewRateLimiter(0.75);
    private SlewRateLimiter leftAccelerationLimit = new SlewRateLimiter(0.75);
    private int pipeline;

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
        Map.entry(8, -120.0)
    );

    /** Creates a new AlignWithLimelight. */
    public AlignWithLimelight(
        Vision vision,
        CommandSwerveDrivetrain driveTrain,
        double goalTY,
        double goalTX,
        double maxSpeed,
        int pipeline,
        XboxController driverCont
    ) {
        this.vision = vision;
        this.driveTrain = driveTrain;
        this.goalTY = goalTY;
        this.goalTX = goalTX;
        this.pipeline = pipeline;
        this.driverCont = driverCont;
        addRequirements(driveTrain);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        leftAccelerationLimit.reset(0);
        forwardsAccelerationLimit.reset(0);

        vision.setPipeline(pipeline);

        double angleToFace = driveTrain.getAngle();
        int priorityID = vision.getAprilTagID();

        endEarly = true;
        if (vision.getTV() == 1) {
            endEarly = false;
            double angle = tagIDToAngle.get(priorityID);
            angleToFace = Objects.nonNull(angle) ? angle : driveTrain.getAngle(); //TODO: fix this?? 2/7
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

        Logger.recordOutput("AlignWLimelight AngleToFace", angleToFace);
        Logger.recordOutput("AlignWLimelight GoalTx", goalTX);
        Logger.recordOutput("AlignWLimelight GoalTy", goalTY);

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

        double velX = -driveTrain.forwardController.calculate(
            vision.getTYRaw(),
            goalTY,
            driveTrain.getState().Timestamp
        );

        double velY = driveTrain.strafeController.calculate(
            vision.getTXRaw(),
            goalTX,
            driveTrain.getState().Timestamp
        );

        Logger.recordOutput("AlignWLimelight PID OutputX", velX);
        Logger.recordOutput("AlignWLimelight PID OutputY", velY);

        Translation2d PIDSpeed = new Translation2d(
            forwardsAccelerationLimit.calculate(velX),
            leftAccelerationLimit.calculate(velY)
        );

        Translation2d rotatedPIDSpeeds = rotateTranslation(PIDSpeed, angleToFaceRotation2d);

        rotatedVelocityX = rotatedPIDSpeeds.getX();
        rotatedVelocityY = rotatedPIDSpeeds.getY();

        Logger.recordOutput("AlignWLimelight BaseOutput", PIDSpeed);
        Logger.recordOutput("AlignWLimelight RotatedOutput", rotatedPIDSpeeds);

        if (vision.getTV() == 1) {
            driveTrain.driveFieldCentricFacingAngle(
                rotatedVelocityX, // forward & backward motion
                rotatedVelocityY,
                angleToFaceRotation2d.getDegrees() // side to side motion
            );
        } else {
            driveTrain.driveFieldCentricFacingAngle(0.0, 0.0, angleToFaceRotation2d.getDegrees());
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    //   LimelightHelpers.SetFiducialIDFiltersOverride("limelight", new int[] { 6, 7, 8 });
        LimelightHelpers.setPriorityTagID("limelight", -1);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return endEarly ; //TODO: THIS DONEST END
    }
}
