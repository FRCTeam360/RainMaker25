// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Servo;

import edu.wpi.first.hal.HALUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Team360SubSystemBase;

import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class Servo extends Team360SubSystemBase {
  private final ServoIO io;
  private final ServoIOInputsAutoLogged inputs = new ServoIOInputsAutoLogged();

  /** Creates a new Servo. */
  public Servo(ServoIO io) {
    this.io = io;
  }

  public void setSpeed(double speed) {
    io.setServoSpeed(speed);
  }

  public Command setSpeedCmd(double speed) {
    return setSpeedCmd(() -> speed);
  }

  public void stop() {
    io.setServoSpeed(0.5);
  }

  public Command runWithTimeout(double timeout, double speed) {
    return Commands.waitSeconds(timeout).deadlineFor(this.setSpeedCmd(speed));
  }

  public Command setSpeedCmd(DoubleSupplier speed) {
    System.out.println("speed");
    System.out.println(speed.getAsDouble());
    return this.runEnd(() -> io.setServoSpeed(speed.getAsDouble()), () -> io.setServoSpeed(0.5));
  }

  @Override
  public void periodic() {
    long periodicStartTime = HALUtil.getFPGATime();
    io.updateInputs(inputs);
    Logger.processInputs(getLogPreFix(), inputs);
    long periodicLoopTime = HALUtil.getFPGATime() - periodicStartTime;
    Logger.recordOutput(getLogPreFix() + "periodic loop time", (periodicLoopTime / 1000.0));
    // This method will be called once per scheduler run
  }
}
