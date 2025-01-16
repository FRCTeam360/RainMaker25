// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Elevator;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.WoodbotConstants;

public class ElevatorIOSim implements ElevatorIO {
  /** Creates a new ElevatorIOSim. */

  private DCMotorSim simMotor = new DCMotorSim(null, DCMotor.getFalcon500(1), null);

  public ElevatorIOSim() {
  }

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {
    simMotor.update(0.02);
    inputs.elevatorPosition = simMotor.getAngularPositionRad();
  }

  public void setElevatorPostion(double height) {
    simMotor.setAngle(height);
  }

  public void setDutyCycle(double dutyCycle) {
    simMotor.setAngularVelocity(dutyCycle);
  }

  
}
