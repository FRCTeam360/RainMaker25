// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Vision;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.List;
import java.util.Optional;
import java.util.OptionalDouble;
import java.util.function.Consumer;

import org.littletonrobotics.junction.Logger;

import com.google.flatbuffers.Table;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.math.InterpolatingMatrixTreeMap;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Vision extends SubsystemBase {
  private NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
  private VisionIO io;
  private VisionIOInputsAutoLogged inputs = new VisionIOInputsAutoLogged();
  private final VisionIO[] ios;
  private final VisionIOInputsAutoLogged[] visionInputs;
  private Timer snapshotTimer = new Timer();
  List<VisionMeasurement> acceptedMeasurements = Collections.emptyList();


  private final String VISION_LOGGING_PREFIX = "Vision: ";

  private static final InterpolatingMatrixTreeMap<Double, N3, N1> MEASUREMENT_STD_DEV_DISTANCE_MAP = new InterpolatingMatrixTreeMap<>();

  static {
    MEASUREMENT_STD_DEV_DISTANCE_MAP.put(1.0, VecBuilder.fill(1.0, 1.0, 1.0));
    MEASUREMENT_STD_DEV_DISTANCE_MAP.put(8.0, VecBuilder.fill(10.0, 10.0, 10.0));
  }

  /** Creates a new Vision. */
  public Vision(VisionIO[] visionIos) {
    this.ios = visionIos;
    // Creates the same number of inputs as vision IO layers
    visionInputs = new VisionIOInputsAutoLogged[visionIos.length];
    Arrays.fill(visionInputs, new VisionIOInputsAutoLogged());
  }

  public int getAprilTagID() {
    return io.getAprilTagID();
  }

  public double getTXRaw() {
    // TODO: replace with more robust code
    return ios[0].getTXRaw();
  }

  public double getTXAdjusted() {
    // TODO: replace with more robust code
    return ios[0].getTXAdjusted();
  }

  public double getTYRaw() {
    // TODO: replace with more robust code
    return ios[0].getTYRaw();
  }

  public double getTYAdjusted() {
    // TODO: replace with more robust code
    return ios[0].getTYAdjusted();
  }

  public double getTV() {
    // TODO: replace with more robust code
    return ios[0].getTV();
  }

  public double getPipeline() {
    // TODO: replace with more robust code
    return ios[0].getPipeline();
  }

  public void setPipeline(int pipeline) {
    // TODO: replace with more robust code
    if (ios[0].getPipeline() != pipeline) {
      ios[0].setPipeline(pipeline);
    }
  }

  public Command waitUntilTargetTxTy(double goalTX, double goalTY) {
    return Commands.waitUntil(() -> isTargetInView() && isOnTargetTX(goalTX) && isOnTargetTY(goalTY));
  }

  public void takeSnapshot() {
    // TODO: replace with more robust code
    ios[0].takeSnapshot();
    Logger.recordOutput(VISION_LOGGING_PREFIX + "snapshot", true);
    snapshotTimer.stop();
    snapshotTimer.reset();
    snapshotTimer.start();
  }

  public void resetSnapshot() {
    // TODO: replace with more robust code
    ios[0].resetSnapshot();
    Logger.recordOutput(VISION_LOGGING_PREFIX + "snapshot", false);
    snapshotTimer.stop();
  }

  public boolean isOnTargetTX(double goal) {
    if (Math.abs(getTXRaw() - goal) < 1.0) {
      return true;
    }
    return false;
  }

  public boolean isOnTargetTY(double goal) {
    if (Math.abs(getTYRaw() - goal) < 1.0) {
      return true;
    }

    return false;
  }

  // Returns true if the target is in view
  public boolean isTargetInView() {
    // TODO: replace with more robust code
    return getTV() == 1;
  }

  @Override
  public void periodic() {
    for (int i = 0; i < ios.length; i++) {
      VisionIO io = ios[i];
      VisionIOInputsAutoLogged input = visionInputs[i];

      io.updateInputs(input);
      Logger.processInputs("Limelight", input);
    }

    List<VisionMeasurement> acceptedMeasurements = new ArrayList<>();

    for (VisionIOInputsAutoLogged input : visionInputs) {
      // skip input if not updated
      if (!input.poseUpdated)
        continue;

      Pose2d pose = input.estimatedPose;
      double timestamp = input.timestampSeconds;

      // Skip measurements that are not with in the field boundary
      if (pose.getX() < 0.0 || pose.getX() > Constants.FIELD_LAYOUT.getFieldLength() ||
          pose.getY() < 0.0 || pose.getY() > Constants.FIELD_LAYOUT.getFieldWidth())
        continue;

      // get standard deviation based on distance to nearest tag
      OptionalDouble closestTagDistance = Arrays.stream(input.distancesToTargets).min();

      Matrix<N3, N1> stdDevs = MEASUREMENT_STD_DEV_DISTANCE_MAP.get(closestTagDistance.orElse(Double.MAX_VALUE));

      acceptedMeasurements.add(new VisionMeasurement(timestamp, pose, stdDevs));
    }
    this.acceptedMeasurements = acceptedMeasurements;
  }

  /**
   * @return Command that consumes vision measurements
   */
  public Command consumeVisionMeasurements(Consumer<List<VisionMeasurement>> visionMeasurementConsumer) {
    return run(() -> visionMeasurementConsumer.accept(acceptedMeasurements));
  }
}
