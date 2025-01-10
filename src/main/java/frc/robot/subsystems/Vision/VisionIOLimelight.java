// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class VisionIOLimelight implements VisionIO {
  private final NetworkTable table;
  private final double height;
  private final double pitch;
  private final double heightFudgeFactor;
  private final double pitchFudgeFactor;

  /**
   * Creates a new Limelight hardware layer.
   * 
   * @param name              the name of the limelight
   * @param height            as designed camera height in meters
   * @param pitch             as designed camera pitch in degrees
   * @param heightFudgeFactor fudge factor for camera height in meters
   * @param pitchFudgeFactor  fudge factor for camera pitch in degrees
   */
  public VisionIOLimelight(String name, double height, double pitch, double heightFudgeFactor,
      double pitchFudgeFactor) {
    table = NetworkTableInstance.getDefault().getTable(name);
    this.height = height;
    this.pitch = pitch;
    this.heightFudgeFactor = heightFudgeFactor;
    this.pitchFudgeFactor = pitchFudgeFactor;
  }

  public void updateInputs(VisionIOInputs inputs) {
    inputs.tv = getTV();
    inputs.tx = getTX();
    inputs.tyBase = getTYBase();
    inputs.tyAdjusted = getTYAdjusted();
    inputs.pipeline = getPipeline();
  }

  public double getTX() {
    return table.getEntry("tx").getDouble(0);
  }

  /**
   * @return the ty without the angle fudge factor
   */
  public double getTYBase() {
    return table.getEntry("ty").getDouble(0);
  }

  public double getTYAdjusted() { 
    return getTYBase() - pitchFudgeFactor;
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

  public void setLEDMode(int mode) {
    table.getEntry("ledMode").setNumber(mode);
  }

  public void takeSnapshot() {
    table.getEntry("snapshot").setNumber(1);
  }

  public void resetSnapshot() {
    table.getEntry("snapshot").setNumber(0);
  }
}
