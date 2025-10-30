// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.AlgaeRoller;

import edu.wpi.first.hal.HALUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Team360SubSystemBase;
import org.littletonrobotics.junction.Logger;

public class AlgaeRoller extends Team360SubSystemBase {
  private final AlgaeRollerIO io;
  private final AlgaeRollerIOInputsAutoLogged inputs = new AlgaeRollerIOInputsAutoLogged();

  /** Creates a new AlgaeIntakeRoller. */
  public AlgaeRoller(AlgaeRollerIO io) {
    this.io = io;
  }

  public void setDutyCycle(double duty) {
    io.setDutyCycle(duty);
  }

  public void stop() {
    io.setDutyCycle(0.0);
  }

  public Command setDutyCycleCmd(double duty) {
    return this.runEnd(() -> io.setDutyCycle(duty), () -> io.setDutyCycle(0.0));
  }

  @Override
  public void periodic() {
    long periodicStartTime = HALUtil.getFPGATime();
    io.updateInputs(inputs);
    Logger.processInputs(getName(), inputs);
    long periodicLoopTime = HALUtil.getFPGATime() - periodicStartTime;
    Logger.recordOutput(getLogPreFix() + "periodic loop time", (periodicLoopTime / 1000.0));
  }
}
