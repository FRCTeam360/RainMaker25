// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.AlgaeShooter;

import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.motorcontrol.PWMTalonFX;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.PWMSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.CoralShooter.CoralShooterIO.CoralShooterIOInputs;

public class AlgaeShooterIOSim implements AlgaeShooterIO {
  private DCMotor gearbox = DCMotor.getKrakenX60Foc(1);
  private Encoder encoder = new Encoder(7, 8);

  private final PWMTalonFX motor = new PWMTalonFX(4);

  private final LinearSystem<N1, N1, N1> plant = LinearSystemId.createFlywheelSystem(
      gearbox, 0.00113951385, 1.0); // TODO: find actual MOI

  private final FlywheelSim algaeShooterSim = new FlywheelSim(
      plant, //
      gearbox, // gearbox
      0.01);

  private final EncoderSim simEncoder = new EncoderSim(encoder);
  private final PWMSim simMotor = new PWMSim(motor);

  /** Creates a new AlgaeShooterIOSim. */
  public AlgaeShooterIOSim() {
    simEncoder.setDistancePerPulse(2.0 * Math.PI * (Units.inchesToMeters(2.0)) / 4096);
    algaeShooterSim.setInput(simMotor.getSpeed() * RobotController.getBatteryVoltage());
    algaeShooterSim.update(0.02);
    simEncoder.setDistance(simMotor.getPosition());
    RoboRioSim.setVInVoltage(BatterySim.calculateDefaultBatteryLoadedVoltage(algaeShooterSim.getCurrentDrawAmps()));
  }

  public void updateInputs(AlgaeShooterIOInputs inputs) {
    inputs.algaeShooterFrontVoltage = algaeShooterSim.getInputVoltage();
    inputs.algaeShooterFrontPosition = simMotor.getPosition();
    inputs.algaeShooterFrontVelocity = algaeShooterSim.getAngularVelocityRPM();
  }

  public void setDutyCycle(double dutyCycle) {
    simMotor.setSpeed(dutyCycle);
  }

  @Override
  public void setVelocity(double velocity) {
    algaeShooterSim.setAngularVelocity(velocity);
  }


  public void stop() {
    simMotor.setSpeed(0.0);
  }
}
