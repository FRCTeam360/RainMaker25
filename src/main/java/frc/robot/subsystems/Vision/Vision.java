// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Vision;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Optional;
import java.util.OptionalDouble;
import java.util.function.Consumer;
import java.util.function.DoubleSupplier;

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
  private VisionIOInputsAutoLogged inputs = new VisionIOInputsAutoLogged();
  //private final VisionIO[] ios;
  private final Map<String, VisionIO> ios;

  private final Map<String, VisionIOInputsAutoLogged> visionInputs;
  private Timer snapshotTimer = new Timer();
  List<VisionMeasurement> acceptedMeasurements = Collections.emptyList();


  private final String VISION_LOGGING_PREFIX = "Vision: ";

  private static final InterpolatingMatrixTreeMap<Double, N3, N1> MEASUREMENT_STD_DEV_DISTANCE_MAP = new InterpolatingMatrixTreeMap<>();

  static {
    MEASUREMENT_STD_DEV_DISTANCE_MAP.put(1.0, VecBuilder.fill(1.0, 1.0, 1.0));
    MEASUREMENT_STD_DEV_DISTANCE_MAP.put(8.0, VecBuilder.fill(10.0, 10.0, 10.0));
  }

  private static final Matrix<N3, N1> stdDevMatrix = VecBuilder.fill(.7, .7, 999999);

  /** Creates a new Vision. */
  public Vision( Map<String, VisionIO> visionIos) {
    this.ios = visionIos;
    // Creates the same number of inputs as vision IO layers
    visionInputs = Map.of();
    for(String key: visionIos.keySet()){
      visionInputs.put(key, new VisionIOInputsAutoLogged());
    }
  }

  public int getAprilTagID(String name) {
    return ios.get(name).getAprilTagID();
  }

  public double getTXRaw(String name) {
    // TODO: replace with more robust code
    return ios.get(name).getTXRaw();
  }


  public double getTYRaw(String name) {
    // TODO: replace with more robust code
    return ios.get(name).getTYRaw();
  }

  public double getTV(String name) {
    // TODO: replace with more robust code
    return ios.get(name).getTV();
  }

  public double getPipeline(String name) {
    // TODO: replace with more robust code
    return ios.get(name).getPipeline();
  }

  public void setPipeline(String name, int pipeline) {
    // TODO: replace with more robust code
    if (ios.get(name).getPipeline() != pipeline) {
      ios.get(name).setPipeline(pipeline);
    }
  }
  
  public void takeSnapshot(String name) {
    // TODO: replace with more robust code
    ios.get(name).takeSnapshot();
    Logger.recordOutput(VISION_LOGGING_PREFIX + "snapshot", true);
    snapshotTimer.stop();
    snapshotTimer.reset();
    snapshotTimer.start();
  }

  public void resetSnapshot(String name) {
    // TODO: replace with more robust code
    ios.get(name).resetSnapshot();
    Logger.recordOutput(VISION_LOGGING_PREFIX + "snapshot", false);
    snapshotTimer.stop();
  }

  public boolean isOnTargetTX(String name, double goal) {
    if (Math.abs(getTXRaw(name) - goal) < 1.0) {
      return true;
    }
    return false;
  }

  public boolean isOnTargetTY(String name, double goal) {
    if (Math.abs(getTYRaw(name) - goal) < 1.0) {
      return true;
    }
    return false;
  }
  
  public boolean isTargetInView(String name) {
    // TODO: replace with more robust code
    return getTV(name) == 1;
  }
  
  public Command waitUntilTargetTxTy(String name, double goalTX, double goalTY) {
    return Commands.waitUntil(() -> isTargetInView(name) && isOnTargetTX(name, goalTX) && isOnTargetTY(name, goalTY));
  }
  @Override
  public void periodic() {
    for (String key : ios.keySet()) {
      VisionIO io = ios.get(key);
      VisionIOInputsAutoLogged input = visionInputs.get(key);

      io.updateInputs(input);
      Logger.processInputs("Limelight: " + key, input);
    }

    List<VisionMeasurement> acceptedMeasurements = new ArrayList<>();

    for (String key: visionInputs.keySet()) {
      VisionIOInputsAutoLogged input = visionInputs.get(key);
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

      acceptedMeasurements.add(new VisionMeasurement(timestamp, pose, stdDevMatrix));
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
