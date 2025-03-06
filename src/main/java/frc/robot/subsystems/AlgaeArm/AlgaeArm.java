// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.AlgaeArm;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DutyCycle;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.CommandLogger;

public class AlgaeArm extends SubsystemBase {

    private DoubleSupplier elevatorHeight;
  private final AlgaeArmIO io;
  private final AlgaeArmIOInputsAutoLogged inputs = new AlgaeArmIOInputsAutoLogged();

  private final LoggedMechanism2d mech2d = new LoggedMechanism2d(30, 50, new Color8Bit(Color.kPapayaWhip));
  private final LoggedMechanismRoot2d mechRoot2D = mech2d.getRoot("algae arm root", 18, 0);
  private final LoggedMechanismLigament2d mechLigiment2D = mechRoot2D.append(new LoggedMechanismLigament2d("algae arm",5 , 0,5, new Color8Bit(Color.kTomato)));

  /** Creates a new AlgaeArm. */
  public AlgaeArm(AlgaeArmIO io, DoubleSupplier elevatorHeight) {
    this.io = io;
    this.elevatorHeight = elevatorHeight;
  }

  public void setPosition(double position) {
    io.setPosition(position);
  }

  public void setDutyCycle(double dutyCycle) {
    io.setDutyCycle(dutyCycle);
  }

  public void stop(){
    io.setDutyCycle(0.0);
  }

  /**
   * method for setting encoder to a new value (i.e. for zeroing)
   * @param value new value in motor rotations
   */
  public void setEncoder(double value) {
    io.setEncoder(value);
  }

  public double getCurrent() {
    return inputs.algaeArmCurrent;
  }

  public Command setDutyCycleCmd(DoubleSupplier dutyCycleSupplier) {
    return this.runEnd(
        () -> this.setDutyCycle(dutyCycleSupplier.getAsDouble()),
        () -> this.setDutyCycle(0.0));
  }

  public Command setDutyCycleCmd(double dutyCycle) {
    return setDutyCycleCmd(() -> dutyCycle);
  }

  public Command setAlgaeArmAngleCmd(double angle) {
    return CommandLogger.logCommand( this.runEnd(
        () -> this.setPosition(angle),
        () -> this.setPosition(angle)), "set algae arm angle cmd");
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    mechLigiment2D.setAngle(new Rotation2d(inputs.algaeArmAngle));
    mechRoot2D.setPosition(19.5, elevatorHeight.getAsDouble() - 3.5);
    io.updateInputs(inputs);
    Logger.processInputs("Algae Arm", inputs);
  }
}
