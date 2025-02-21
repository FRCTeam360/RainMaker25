// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.ClimberWinch;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.PWMSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.CoralShooter.CoralShooterIO.CoralShooterIOInputs;

public class ClimberWinchIOSim implements ClimberWinchIO {

  private DCMotor gearbox = DCMotor.getNEO(1);
  private Encoder winchEncoder = new Encoder(9, 10);

  private final PWMSparkMax winchMotor = new PWMSparkMax(7);

  private final LinearSystem<N1, N1, N1> plant = LinearSystemId.createFlywheelSystem(
      gearbox, 0.00113951385, 1.0); // TODO: find actual MOI

  private final FlywheelSim climberSim = new FlywheelSim(
      plant, //
      gearbox, // gearbox
      0.01);

  private final EncoderSim simWinchEncoder = new EncoderSim(winchEncoder);
  private final PWMSim simWinchMotor = new PWMSim(winchMotor);

  /** Creates a new ClimberIOSim. */
  public ClimberWinchIOSim() {}

  public void updateInputs(ClimberWinchIOInputs inputs) {
    simWinchEncoder.setDistancePerPulse(2.0 * Math.PI * (Units.inchesToMeters(2.0)) / 4096);
    climberSim.setInput(simWinchMotor.getSpeed() * RobotController.getBatteryVoltage());
    climberSim.update(0.02);
    simWinchEncoder.setDistance(simWinchMotor.getPosition());

    RoboRioSim.setVInVoltage(BatterySim.calculateDefaultBatteryLoadedVoltage(climberSim.getCurrentDrawAmps()));

    inputs.winchPosition = simWinchMotor.getPosition();
    inputs.winchVelocity = climberSim.getAngularVelocityRPM();
    inputs.winchDutyCycle = climberSim.getInputVoltage();
  }


  @Override
  public void setDutyCycle(double dutyCycle) {
    simWinchMotor.setSpeed(dutyCycle);
  }

  @Override
  public void setPosition(double position) {
    simWinchMotor.setPosition(position);
  }
}
