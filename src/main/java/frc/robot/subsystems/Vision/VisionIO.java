// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Vision;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;

public interface VisionIO {
  /** Creates a new VisionIO. */
  @AutoLog
    public static class VisionIOInputs {
        public double tx;
        public double txAdjusted;
        public double ty;
        public double tyAdjusted;
        public double tv;
        public double pipeline;

        public boolean connected = false;
        public TargetObservation latestTargetObservation =
            new TargetObservation(new Rotation2d(), new Rotation2d());
        public PoseObservation[] poseObservations = new PoseObservation[0];
        public int[] tagIds = new int[0];
        
    }

    public void updateInputs(VisionIOInputs inputs);

    public double getTXRaw();

    public double getTXAdjusted();

    public double getTYRaw();

    public double getTYAdjusted();

    public double getTV();

    public Pose2d getBotPose();

    public double getPipeline();

    public void setPipeline(int pipeline);

    public void takeSnapshot();

    public void resetSnapshot();

      public static record TargetObservation(Rotation2d tx, Rotation2d ty) {}

  /** Represents a robot pose sample used for pose estimation. */
  public static record PoseObservation(
      double timestamp,
      Pose3d pose,
      double ambiguity,
      int tagCount,
      double averageTagDistance,
      PoseObservationType type) {}

  public static enum PoseObservationType {
    MEGATAG_1,
    MEGATAG_2,
    PHOTONVISION
  }

}
