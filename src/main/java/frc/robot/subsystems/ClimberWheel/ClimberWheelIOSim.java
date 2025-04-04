// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.ClimberWheel;

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
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.ClimberWheel.ClimberWheelIO.ClimberWheelIOInputs;

public class ClimberWheelIOSim implements ClimberWheelIO {
  private final DCMotor gearbox = DCMotor.getNEO(1);
  private final Encoder encoder = new Encoder(10, 11);

  private final PWMSparkMax wheelMotor = new PWMSparkMax(6);

  private final LinearSystem<N1, N1, N1> plant = LinearSystemId.createFlywheelSystem(
      gearbox, 0.00113951385, 1.0); // TODO: find actual MOI

  private final FlywheelSim climberSim = new FlywheelSim(
      plant, //
      gearbox, // gearbox
      0.01);

  private final EncoderSim simWheelEncoder = new EncoderSim(encoder);
  private final PWMSim simWheelMotor = new PWMSim(wheelMotor);

  /** Creates a new ClimberWheelIOSim. */
  public ClimberWheelIOSim() {}

  public void setDutyCycle(double dutyCycle) {
    simWheelMotor.setSpeed(dutyCycle);
}

  public void updateInputs(ClimberWheelIOInputs inputs) {
    simWheelEncoder.setDistancePerPulse(2.0 * Math.PI * (Units.inchesToMeters(2.0)) / 4096);
    climberSim.setInput(simWheelMotor.getSpeed() * RobotController.getBatteryVoltage());
    climberSim.update(0.02);
    simWheelEncoder.setDistance(simWheelMotor.getPosition());
    RoboRioSim.setVInVoltage(BatterySim.calculateDefaultBatteryLoadedVoltage(climberSim.getCurrentDrawAmps()));

    inputs.wheelPosition = simWheelMotor.getPosition();
    inputs.wheelVelocity = climberSim.getAngularVelocityRPM();
    inputs.wheelDutyCycle = climberSim.getInputVoltage();
  }
}
