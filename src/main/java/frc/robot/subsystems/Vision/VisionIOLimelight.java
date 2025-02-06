// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Vision;

import java.lang.reflect.Array;
import java.util.Optional;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.utils.LimelightHelpers;
import frc.robot.utils.LimelightHelpers.PoseEstimate;
import frc.robot.utils.LimelightHelpers.RawFiducial;

public class VisionIOLimelight implements VisionIO {
  private final NetworkTable table;
  private final String name;
  private final double yawFudgeFactor;
  private final double pitchFudgeFactor;
  private final DoubleSupplier gyroAngleSupplier;

  private RawFiducial[] fiducials = LimelightHelpers.getRawFiducials("");

  /**
   * Creates a new Limelight hardware layer.
   * 
   * @param name              the name of the limelight
   * @param yawFudgeFactor    fudge factor for camera yaw in degrees
   * @param pitchFudgeFactor  fudge factor for camera pitch in degrees
   */
  public VisionIOLimelight(String name, double yawFudgeFactor, double pitchFudgeFactor, DoubleSupplier gyroAngleSupplier) {
    table = NetworkTableInstance.getDefault().getTable(name);
    this.name = name;
    this.yawFudgeFactor = yawFudgeFactor;
    this.pitchFudgeFactor = pitchFudgeFactor;
    this.gyroAngleSupplier = gyroAngleSupplier;
  }

  public void updateInputs(VisionIOInputs inputs) {
    // Get the pose estimate from limelight helpers
    Optional<PoseEstimate> newPoseEstimate = getMegatagPoseEst();

    // Assume that the pose hasn't been updated
    inputs.poseUpdated = false;

    inputs.tv = getTV();
    inputs.tx = getTXRaw();
    inputs.txAdjusted = getTXAdjusted();
    inputs.ty = getTYRaw();
    inputs.tyAdjusted = getTYAdjusted();
    inputs.pipeline = getPipeline();
    inputs.tagID = getAprilTagID();
    // if the new pose estimate is null, then don't update further
    if(newPoseEstimate.isEmpty()) return;

    PoseEstimate poseEstimate = newPoseEstimate.get();

    inputs.estimatedPose = poseEstimate.pose;
    inputs.timestampSeconds = poseEstimate.timestampSeconds;
    int[] targetIds = new int[poseEstimate.rawFiducials.length];
    double[] distancesToTargets = new double[poseEstimate.rawFiducials.length];
    for (int i = 0; i < poseEstimate.rawFiducials.length; i++){
      RawFiducial rawFiducial = poseEstimate.rawFiducials[i];
      targetIds[i] = rawFiducial.id;
      distancesToTargets[i] = rawFiducial.distToRobot;
    }
    inputs.targetIds = targetIds;
    inputs.distancesToTargets = distancesToTargets;
    inputs.poseUpdated = true;
  }

  private Optional<PoseEstimate> getMegatagPoseEst(){
    LimelightHelpers.SetRobotOrientation(name, gyroAngleSupplier.getAsDouble(), 0, 0, 0, 0, 0);
    PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(name);
    return Optional.ofNullable(mt2);
  }

  }
  public int getAprilTagID() {
    return (int)table.getEntry("tid").getInteger(0);
  }
  
  public double getTXRaw() {
    return table.getEntry("tx").getDouble(0.0);
  }

  public double getTXAdjusted() {
    return getTXRaw() - yawFudgeFactor;
  }

  public double getTYRaw() {
    return table.getEntry("ty").getDouble(0);
  }

  public double getTYAdjusted() {
    return getTYRaw() - pitchFudgeFactor;
  }

  public double getTV() {
    return table.getEntry("tv").getDouble(0);
  }

  private boolean targetInView() {
    return getTV() == 1.0;
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
