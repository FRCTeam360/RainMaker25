// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Climber;

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

public class ClimberIOSim implements ClimberIO {

  private DCMotor gearbox = DCMotor.getNEO(2);
  private Encoder winchEncoder = new Encoder(8, 9);
  private Encoder wheelEncoder = new Encoder(10, 11);

  private final PWMSparkMax winchMotor = new PWMSparkMax(5);
  private final PWMSparkMax wheelMotor = new PWMSparkMax(6);

  private final LinearSystem<N1, N1, N1> plant = LinearSystemId.createFlywheelSystem(
      gearbox, 0.00113951385, 1.0); // TODO: find actual MOI

  private final FlywheelSim climberSim = new FlywheelSim(
      plant, //
      gearbox, // gearbox
      0.01);

  private final EncoderSim simWinchEncoder = new EncoderSim(winchEncoder);
  private final EncoderSim simWheelEncoder = new EncoderSim(wheelEncoder);
  private final PWMSim simWinchMotor = new PWMSim(winchMotor);
  private final PWMSim simWheelMotor = new PWMSim(wheelMotor);

  /** Creates a new ClimberIOSim. */
  public ClimberIOSim() {
  }

  public void updateInputs(ClimberIOInputs inputs) {
    simWinchEncoder.setDistancePerPulse(2.0 * Math.PI * (Units.inchesToMeters(2.0)) / 4096);
    climberSim.setInput(simWinchMotor.getSpeed() * RobotController.getBatteryVoltage());
    climberSim.update(0.02);
    simWinchEncoder.setDistance(simWinchMotor.getPosition());

    simWheelEncoder.setDistancePerPulse(2.0 * Math.PI * (Units.inchesToMeters(2.0)) / 4096);
    climberSim.setInput(simWheelMotor.getSpeed() * RobotController.getBatteryVoltage());
    climberSim.update(0.02);
    simWheelEncoder.setDistance(simWheelMotor.getPosition());

    RoboRioSim.setVInVoltage(BatterySim.calculateDefaultBatteryLoadedVoltage(climberSim.getCurrentDrawAmps()));

    inputs.winchPosition = simWinchMotor.getPosition();
    inputs.winchVelocity = climberSim.getAngularVelocityRPM();
    inputs.winchDutyCycle = climberSim.getInputVoltage();

    inputs.wheelPosition = simWheelMotor.getPosition();
    inputs.wheelVelocity = climberSim.getAngularVelocityRPM();
    inputs.wheelDutyCycle = climberSim.getInputVoltage();
  }


  @Override
  public void setWinchDutyCycle(double dutyCycle) {
    simWinchMotor.setSpeed(dutyCycle);
  }

  @Override
  public void setWheelDutyCycle(double dutyCycle) {
    simWheelMotor.setSpeed(dutyCycle);
  }
}
