// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Vision;

import java.util.Optional;

import org.littletonrobotics.junction.Logger;

import com.google.flatbuffers.Table;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Vision extends SubsystemBase {
  private NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
  private VisionIO io;
  private VisionIOInputsAutoLogged inputs = new VisionIOInputsAutoLogged();
  private Timer snapshotTimer = new Timer();

  private final String VISION_LOGGING_PREFIX = "Vision: ";

  /** Creates a new Vision. */
  public Vision(VisionIO io) {
    this.io = io;
  }

  public int getAprilTagID() {
    return io.getAprilTagID();
  }

  public double getTXRaw() {
    return io.getTXRaw();
  }

  public double getTXAdjusted() {
    return io.getTXAdjusted();
  }

  public double getTYRaw() {
    return io.getTYRaw();
  }

  public double getTYAdjusted() {
    return io.getTYAdjusted();
  }

  public double getTV() {
    return io.getTV();
  }

  public double getPipeline() {
    return io.getPipeline();
  }

  public Pose2d getBotPose() {
    return io.getBotPose();
  }

  public void setPipeline(int pipeline) {
    if (io.getPipeline() != pipeline) {
      io.setPipeline(pipeline);
    }
  }

  public void takeSnapshot() {
    io.takeSnapshot();
    Logger.recordOutput(VISION_LOGGING_PREFIX + "snapshot", true);
    snapshotTimer.stop();
    snapshotTimer.reset();
    snapshotTimer.start();
  }

  public void resetSnapshot() {
    io.resetSnapshot();
    Logger.recordOutput(VISION_LOGGING_PREFIX + "snapshot", false);
    snapshotTimer.stop();
  }

  public boolean isOnTargetTX(double goal) {
    if (Math.abs(getTXAdjusted()) < goal) {
      return true;
    }
    return false;
  }

  public boolean isOnTargetTY(double goal) {
    if (Math.abs(getTYAdjusted()) < goal) {
      return true;
    }

    return false;
  }

  // Returns true if the target is in view
  public boolean isTargetInView() {
    return getTV() == 1;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Limelight", inputs);
  }
}
