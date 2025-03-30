// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.hal.HALUtil;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.Constants.PracticeBotConstants;
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
    private Rotation2d angleToFaceRotation2d;
    private double rotatedVelocityX;
    private double rotatedVelocityY;
    private boolean endEarly = false;
    private boolean inAuto = false;
    private SlewRateLimiter forwardsAccelerationLimit = new SlewRateLimiter(0.75);
    private SlewRateLimiter leftAccelerationLimit = new SlewRateLimiter(0.75);
    private int pipeline;
    private NetworkTable table = NetworkTableInstance.getDefault().getTable(Constants.CompBotConstants.CORAL_LIMELIGHT_NAME);

    private CommandXboxController driverCont;

    private final String LIMELIGHT_NAME = Constants.PracticeBotConstants.CORAL_LIMELIGHT_NAME;
    private final String CMD_NAME = "AlignWithLimelight: ";

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
        int pipeline,
        CommandXboxController driverCont
    ) {
        this.vision = vision;
        this.driveTrain = driveTrain;
        this.goalTY = goalTY;
        this.goalTX = goalTX;
        this.pipeline = pipeline;
        this.driverCont = driverCont;
        addRequirements(driveTrain);
    }

    /** Creates a new AlignWithLimelight without drivercont */
    public AlignWithLimelight(
        Vision vision,
        CommandSwerveDrivetrain driveTrain,
        double goalTY,
        double goalTX,
        int pipeline
    ) {
        this.vision = vision;
        this.driveTrain = driveTrain;
        this.goalTY = goalTY;
        this.goalTX = goalTX;
        this.pipeline = pipeline;
        addRequirements(driveTrain);

        this.inAuto = true;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        leftAccelerationLimit.reset(0);
        forwardsAccelerationLimit.reset(0);

        vision.setPipeline(LIMELIGHT_NAME, pipeline);

        double angleToFace = driveTrain.getAngle();
        int priorityID = vision.getAprilTagID(LIMELIGHT_NAME);

        endEarly = true;
        if (vision.getTV(LIMELIGHT_NAME) == 1 && tagIDToAngle.containsKey(priorityID)) {
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
        Logger.recordOutput(CMD_NAME + "GoalTx", goalTX);
        Logger.recordOutput(CMD_NAME + "GoalTy", goalTY);

        Logger.recordOutput(CMD_NAME + "endEarly", endEarly);
        LimelightHelpers.setPriorityTagID("limelight", priorityID);

        if (endEarly) return;
        driveRobot();
    }

    public static Translation2d rotateTranslation(
        Translation2d translationToRotate,
        Rotation2d rotation
    ) {
        return translationToRotate.rotateBy(rotation);
    }

    private void driveRobot() {
        double velX = -driveTrain.forwardController.calculate(
            table.getEntry("ty").getDouble(0.0),
            goalTY,
            driveTrain.getState().Timestamp
        );

        double velY = driveTrain.strafeController.calculate(
            table.getEntry("tx").getDouble(0.0),
            goalTX,
            driveTrain.getState().Timestamp
        );

        MathUtil.clamp(velX, 0.025, velX);
        MathUtil.clamp(velY, 0.025, velY); //CLAMP NUMBER FOR HEADINGCONTORLLER IS 0.021


        Logger.recordOutput(CMD_NAME + "PID OutputX", velX);
        Logger.recordOutput(CMD_NAME + "PID OutputY", velY);


        Translation2d PIDSpeed = new Translation2d(
            forwardsAccelerationLimit.calculate(velX),
            leftAccelerationLimit.calculate(velY)
        );

        Translation2d rotatedPIDSpeeds = rotateTranslation(PIDSpeed, angleToFaceRotation2d);

        rotatedVelocityX = rotatedPIDSpeeds.getX();
        rotatedVelocityY = rotatedPIDSpeeds.getY();

        Logger.recordOutput(CMD_NAME + "BaseOutput", PIDSpeed);
        Logger.recordOutput(CMD_NAME + "RotatedOutput", rotatedPIDSpeeds);

        if (vision.getTV(LIMELIGHT_NAME) == 1) {
            driveTrain.driveFieldCentricFacingAngle(
                rotatedVelocityX, // forward & backward motion
                rotatedVelocityY,
                angleToFaceRotation2d.getDegrees() // side to side motion
            );
        } else {
            driveTrain.driveFieldCentricFacingAngle(0.0, 0.0, angleToFaceRotation2d.getDegrees());
        }
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        long executeStartTime = HALUtil.getFPGATime();
        if (endEarly) return;
        driveRobot();
        long executeLoopTime = HALUtil.getFPGATime() - executeStartTime;
        Logger.recordOutput( CMD_NAME +" execute loop time", (executeLoopTime / 1000));
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        LimelightHelpers.setPriorityTagID("limelight", -1);
        driveTrain.robotCentricDrive(0.0, 0.0, 0.0);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        boolean onTX = driveTrain.strafeController.atSetpoint();
        boolean onTY = driveTrain.forwardController.atSetpoint();
        boolean onHeading = driveTrain.isAtRotationSetpoint();
        boolean onCorrectPipeline = (vision.getPipeline(LIMELIGHT_NAME) == pipeline);
        boolean vel0 =
            ((Math.abs(driveTrain.getXRate()) <= 0.1) && (Math.abs(driveTrain.getYRate()) <= 0.1));

        Logger.recordOutput(CMD_NAME + "onTX", onTX);
        Logger.recordOutput(CMD_NAME + "onTY", onTY);
        Logger.recordOutput(CMD_NAME + "setPointTX", goalTX);
        Logger.recordOutput(CMD_NAME + "setPointTY", goalTY);
        Logger.recordOutput(CMD_NAME + "onHeading", onHeading);
        Logger.recordOutput(CMD_NAME + "using correct pipeline", onCorrectPipeline);
        Logger.recordOutput(CMD_NAME + "vel is 0", vel0);
        Logger.recordOutput(CMD_NAME + "iAuto", inAuto);


        return (
            endEarly ||
            (
                vel0 &&
                onCorrectPipeline &&
                onTX &&
                onTY &&
                onHeading &&
                vision.isTargetInView(LIMELIGHT_NAME)
            )
        );
    }
}
