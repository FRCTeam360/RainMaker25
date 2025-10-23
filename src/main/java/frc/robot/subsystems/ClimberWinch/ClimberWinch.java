// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.ClimberWinch;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.hal.HALUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimberWinch extends SubsystemBase {
  private final ClimberWinchIO io;
  private final ClimberWinchIOInputsAutoLogged inputs = new ClimberWinchIOInputsAutoLogged();
  
  /** Creates a new Climber. */
  public ClimberWinch(ClimberWinchIO io) {
    this.io = io;
  }

  public void setDutyCycle(double dutyCycle) {
    io.setDutyCycle(dutyCycle);
  }

  public void stop() {
    io.setDutyCycle(0.0);
  }

  public Command setDutyCycleCmd(double dutyCycle) {
      return this.runEnd(() -> this.setDutyCycle(dutyCycle), () -> this.setDutyCycle(0));
  }

  public Command setDutyCycleCmd(DoubleSupplier dutyCycle) {
    return this.runEnd(() -> this.setDutyCycle(dutyCycle.getAsDouble()), () -> this.setDutyCycle(0));
}

  public void setPosition(double position) {
    io.setPosition(position);
  }
  public double getPosition() {
    return inputs.winchPosition;
  }

  public Command setPositionCmd(double position) {
    return this.runEnd(() -> this.setPosition(position), () -> this.setPosition(position));
  }

  @Override
  public void periodic() {
    long periodicStartTime = HALUtil.getFPGATime();
    io.updateInputs(inputs);
    Logger.processInputs("Climber", inputs);
    long periodicLoopTime = HALUtil.getFPGATime() - periodicStartTime;
    Logger.recordOutput( "Climber: periodic loop time", (periodicLoopTime / 1000.0));
  }
}
