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
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Vision.LimelightHelpers;
import frc.robot.subsystems.Vision.Vision;

import java.util.HashMap;
import java.util.Map;
import java.util.Objects;
import java.util.Optional;
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
    private double lastTXValue;
    private double lastTYValue;
    private Rotation2d angleToFaceRotation2d;
    private double rotatedVelocityX;
    private double rotatedVelocityY;
    private boolean endEarly = false;
    private XboxController driverCont;
    private SlewRateLimiter forwardsAccelerationLimit = new SlewRateLimiter(1);
    private SlewRateLimiter leftAccelerationLimit = new SlewRateLimiter(.3);

    private int[] aprilTagID;

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

    /** Creates a new AlignWithLimelight. */
    public AlignWithLimelight(
            Vision vision,
            CommandSwerveDrivetrain driveTrain,
            double goalTY,
            double goalTX,
            double maxSpeed,
            XboxController driverCont) {
        this.vision = vision;
        this.driveTrain = driveTrain;
        this.goalTY = goalTY;
        this.maxSpeed = maxSpeed;
        this.goalTX = goalTX;
        this.driverCont = driverCont;

        addRequirements(driveTrain);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {

        leftAccelerationLimit.reset(0);
        forwardsAccelerationLimit.reset(0);

        double angleToFace = 0;

        endEarly = true;
        if (vision.getTV() == 1) {
            // aprilTagID[0] = vision.getAprilTagID();
            // LimelightHelpers.SetFiducialIDFiltersOverride("limelight", aprilTagID);
            endEarly = false;

            double angle = tagIDToAngle.get(vision.getAprilTagID());
            angleToFace = Objects.nonNull(angle) ? angle : 0.0;

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
    }

    public static Translation2d rotateTranslation(Translation2d translationToRotate, Rotation2d rotation) {
        return translationToRotate.rotateBy(rotation);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {

        double velX = -driveTrain.translationController.calculate(vision.getTYRaw(), goalTY);
        double velY = 0.05 * driveTrain.translationController.calculate(vision.getTXRaw(), goalTX);
        MathUtil.applyDeadband(velX, 0.04);
     //   double scale = Math.abs(velY / velX);
        // double joystickInput = MathUtil.applyDeadband(driverCont.getLeftY(), 0.1);
        // velX = -Math.signum(joystickInput) * Math.pow(joystickInput, 2);
        Logger.recordOutput("AlignWLimelight PID OutputX", velX);
        Logger.recordOutput("AlignWLimelight PID OutputY", velY);

        Translation2d PIDSpeed = new Translation2d(
                forwardsAccelerationLimit.calculate(velX),
                leftAccelerationLimit.calculate(velY));

        Translation2d rotatedPIDSpeeds = rotateTranslation(PIDSpeed, angleToFaceRotation2d);

        rotatedVelocityX = rotatedPIDSpeeds.getX();
        rotatedVelocityY = rotatedPIDSpeeds.getY();

        Logger.recordOutput("AlignWLimelight BaseOutput", PIDSpeed);
        Logger.recordOutput("AlignWLimelight RotatedOutput", rotatedPIDSpeeds);

        if (vision.getTV() == 1) {
            driveTrain.driveFieldCentricFacingAngle(
                    rotatedVelocityX, // forward & backward motion
                    rotatedVelocityY,
                    angleToFaceRotation2d.getDegrees(), // side to side motion
                    maxSpeed);

        } else {
       driveTrain.driveFieldCentricFacingAngle(0.0, 0.0, angleToFaceRotation2d.getDegrees(), maxSpeed);

        }

    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        // return false;
        return endEarly;// && //Math.abs(vision.getTYRaw() - goalTY) < 0.5; &&
                        // Math.abs(vision.getTXRaw() - goalTX) < 1.0
    }//
}
