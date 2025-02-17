// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Vision;

import java.util.Optional;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.Constants;
import frc.robot.utils.LimelightHelpers;
import frc.robot.utils.LimelightHelpers.PoseEstimate;
import frc.robot.utils.LimelightHelpers.RawFiducial;

public class VisionIOLimelight implements VisionIO {
  private final NetworkTable table;
  private final String name;
  private final double yawFudgeFactor;
  private final double pitchFudgeFactor;
  private final DoubleSupplier gyroAngleSupplier;
  private final DoubleSupplier gryoAngleRateSupplier;

  /**
   * Creates a new Limelight hardware layer.
   * 
   * @param name              the name of the limelight
   * @param yawFudgeFactor    fudge factor for camera yaw in degrees
   * @param pitchFudgeFactor  fudge factor for camera pitch in degrees
   */
  public VisionIOLimelight(String name, double yawFudgeFactor, double pitchFudgeFactor, DoubleSupplier gyroAngleSupplier, DoubleSupplier gryoAngleRateSupplier) {
    table = NetworkTableInstance.getDefault().getTable(name);
    this.name = name;
    this.yawFudgeFactor = yawFudgeFactor;
    this.pitchFudgeFactor = pitchFudgeFactor;
    this.gyroAngleSupplier = gyroAngleSupplier;
    this.gryoAngleRateSupplier = gryoAngleRateSupplier;
  }

  public void updateInputs(VisionIOInputs inputs) {
    // Get the pose estimate from limelight helpers
    Optional<PoseEstimate> newPoseEstimate = getMegatagPoseEst();

    // Assume that the pose hasn't been updated
    inputs.poseUpdated = false;

    inputs.pipeline = getPipeline();

    inputs.tv = getTV();
    inputs.tx = getTXRaw();
    inputs.txAdjusted = getTXAdjusted();
    inputs.ty = getTYRaw();
    inputs.tyAdjusted = getTYAdjusted();
    // if the new pose estimate is null or angle rate is greater than 720 degrees per, then don't update further
    if(inputs.tv == 0.0 || newPoseEstimate.isEmpty() || gryoAngleRateSupplier.getAsDouble() > 720.0) return;

    PoseEstimate poseEstimate = newPoseEstimate.get();

    inputs.estimatedPose = poseEstimate.pose;
    inputs.timestampSeconds = poseEstimate.timestampSeconds;
    int[] targetIds = new int[poseEstimate.rawFiducials.length];
    double[] distancesToTargets = new double[poseEstimate.rawFiducials.length];
    Pose3d[] tagPoses = new Pose3d[poseEstimate.rawFiducials.length];
    for (int i = 0; i < poseEstimate.rawFiducials.length; i++){
      RawFiducial rawFiducial = poseEstimate.rawFiducials[i];
      // if the pose is outside of the field, then skip to the next point
      Optional<Pose3d> tagPose = Constants.FIELD_LAYOUT.getTagPose(rawFiducial.id);
      if(tagPose.isEmpty()) continue;

      targetIds[i] = rawFiducial.id;
      distancesToTargets[i] = rawFiducial.distToRobot;
      tagPoses[i] = tagPose.get();
    }
    inputs.targetIds = targetIds;
    inputs.distancesToTargets = distancesToTargets;
    inputs.tagPoses = tagPoses;
    inputs.poseUpdated = true;
  }

  private Optional<PoseEstimate> getMegatagPoseEst(){
    LimelightHelpers.SetRobotOrientation(name, gyroAngleSupplier.getAsDouble(), gryoAngleRateSupplier.getAsDouble(), 0, 0, 0, 0);
    PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(name);
    return Optional.ofNullable(mt2);
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
