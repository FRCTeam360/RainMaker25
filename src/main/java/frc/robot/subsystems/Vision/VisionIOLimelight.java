// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Vision;

import java.lang.reflect.Array;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.subsystems.Vision.LimelightHelpers.RawFiducial;

public class VisionIOLimelight implements VisionIO {
  private final NetworkTable table;
  private final double yawFudgeFactor;
  private final double pitchFudgeFactor;

  private RawFiducial[] fiducials = LimelightHelpers.getRawFiducials("");

  /**
   * Creates a new Limelight hardware layer.
   * 
   * @param name              the name of the limelight
   * @param yawFudgeFactor    fudge factor for camera yaw in degrees
   * @param pitchFudgeFactor  fudge factor for camera pitch in degrees
   */
  public VisionIOLimelight(String name, double yawFudgeFactor, double pitchFudgeFactor) {
    table = NetworkTableInstance.getDefault().getTable(name);
    this.yawFudgeFactor = yawFudgeFactor;
    this.pitchFudgeFactor = pitchFudgeFactor;
  }

  public void updateInputs(VisionIOInputs inputs) {
    inputs.tv = getTV();
    inputs.tx = getTXRaw();
    inputs.txAdjusted = getTXAdjusted();
    inputs.ty = getTYRaw();
    inputs.tyAdjusted = getTYAdjusted();
    inputs.pipeline = getPipeline();

  }
  public int[] getAprilTagIDs(){
    int[] ids = {5, 6, 7, 8};
    //TODO: code to add apriltag ids to ids[] array
    return ids;
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

  public Pose2d getBotPose() {
    if (targetInView()) {
      double[] botPoseArray = table.getEntry("botpose").getDoubleArray(new double[6]);
      return new Pose2d(botPoseArray[0], botPoseArray[1], Rotation2d.fromDegrees(botPoseArray[5]));
    }
    return null;
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
