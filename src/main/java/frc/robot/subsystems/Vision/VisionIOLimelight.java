// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Vision;

import java.lang.reflect.Array;
import java.util.Optional;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Constants;
import frc.robot.utils.LimelightHelpers;
import frc.robot.utils.LimelightHelpers.PoseEstimate;
import frc.robot.utils.LimelightHelpers.RawFiducial;

public class VisionIOLimelight implements VisionIO {
    private final NetworkTable table;
    private final String name;
    private final DoubleSupplier gyroAngleSupplier;
    private final DoubleSupplier gryoAngleRateSupplier;

    private boolean acceptMeasurements = false;

    private RawFiducial[] fiducials = LimelightHelpers.getRawFiducials("");

    /**
     * Creates a new Limelight hardware layer.
     * 
     * @param name the name of the limelight
     */
    public VisionIOLimelight(String name, DoubleSupplier gyroAngleSupplier, DoubleSupplier gryoAngleRateSupplier,
            boolean acceptMeasurements) {
        table = NetworkTableInstance.getDefault().getTable(name);
        this.name = name;
        this.gyroAngleSupplier = gyroAngleSupplier;
        this.gryoAngleRateSupplier = gryoAngleRateSupplier;
        this.acceptMeasurements = acceptMeasurements;
    }

    /**
     * Creates a new Limelight hardware layer.
     * 
     * @param name the name of the limelight
     */
    public VisionIOLimelight(String name, DoubleSupplier gyroAngleSupplier, DoubleSupplier gryoAngleRateSupplier) {
        table = NetworkTableInstance.getDefault().getTable(name);
        this.name = name;
        this.gyroAngleSupplier = gyroAngleSupplier;
        this.gryoAngleRateSupplier = gryoAngleRateSupplier;
    }

    public void setLEDMode(int mode) {
        table.getEntry("ledMode").setNumber(mode);
      }
    

    public void updateInputs(VisionIOInputs inputs) {

        // Assume that the pose hasn't been updated
        inputs.poseUpdated = false;

        inputs.tv = getTV();
        inputs.tx = getTXRaw();
        inputs.ty = getTYRaw();
        inputs.pipeline = getPipeline();
        inputs.tagID = getAprilTagID();

        if (acceptMeasurements == false) {
            return;
        }

        // Get the pose estimate from limelight helpers
        Optional<PoseEstimate> newPoseEstimate;
        // If enabled, get megatag 2 pose
        newPoseEstimate = getMegatag2PoseEst();

        // if the new pose estimate is null, then don't update further
        if (newPoseEstimate.isEmpty())
            return;
        // if the new pose estimate is null or angle rate is greater than 720 degrees
        // per, then don't update further
        if (inputs.tv == 0.0 || newPoseEstimate.isEmpty() || gryoAngleRateSupplier.getAsDouble() > 720.0)
            return;
        // if the megatag1 pose estimate has less than 2 tags in it, don't update
        // further
        if (!newPoseEstimate.get().isMegaTag2)
            return;
        if(Math.abs(newPoseEstimate.get().pose.getRotation().minus(
            Rotation2d.fromDegrees(gyroAngleSupplier.getAsDouble())
            ).getDegrees()) > 60.0)
            return;
        

        PoseEstimate poseEstimate = newPoseEstimate.get();

        inputs.estimatedPose = poseEstimate.pose;
        inputs.timestampSeconds = poseEstimate.timestampSeconds;
        int[] targetIds = new int[poseEstimate.rawFiducials.length];
        double[] distancesToTargets = new double[poseEstimate.rawFiducials.length];
        Pose3d[] tagPoses = new Pose3d[poseEstimate.rawFiducials.length];
        for (int i = 0; i < poseEstimate.rawFiducials.length; i++) {
            RawFiducial rawFiducial = poseEstimate.rawFiducials[i];
            // if the pose is outside of the field, then skip to the next point
            Optional<Pose3d> tagPose = Constants.FIELD_LAYOUT.getTagPose(rawFiducial.id);
            if (tagPose.isEmpty())
                continue;

            targetIds[i] = rawFiducial.id;
            distancesToTargets[i] = rawFiducial.distToRobot;
            tagPoses[i] = tagPose.get();
        }

        inputs.targetIds = targetIds;
        inputs.distancesToTargets = distancesToTargets;
        inputs.tagPoses = tagPoses;
        inputs.poseUpdated = true;
    }

    private Optional<PoseEstimate> getMegatag2PoseEst() {
        LimelightHelpers.SetRobotOrientation(name, gyroAngleSupplier.getAsDouble(), gryoAngleRateSupplier.getAsDouble(),
                0, 0, 0, 0);
        PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(name);
        return Optional.ofNullable(mt2);
    }

    private Optional<PoseEstimate> getMegatag1PoseEst() {
        LimelightHelpers.SetRobotOrientation(name, gyroAngleSupplier.getAsDouble(), gryoAngleRateSupplier.getAsDouble(),
                0, 0, 0, 0);
        PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue(name);
        return Optional.ofNullable(mt2);
    }

    public int getAprilTagID() {
        return (int) table.getEntry("tid").getInteger(0);
    }

    public double getTXRaw() {
        return table.getEntry("tx").getDouble(0.0);
    }

    public double getTYRaw() {
        return table.getEntry("ty").getDouble(0);
    }

    public double getTV() {
        return table.getEntry("tv").getDouble(0);
    }

    public double getPipeline() {
        return table.getEntry("getpipe").getDouble(0);
    }

    public void setPipeline(int pipeline) {
        table.getEntry("pipeline").setNumber(pipeline);
    }

    public void takeSnapshot() {
        table.getEntry("snapshot").setNumber(1);
    }

    public void resetSnapshot() {
        table.getEntry("snapshot").setNumber(0);
    }
}
