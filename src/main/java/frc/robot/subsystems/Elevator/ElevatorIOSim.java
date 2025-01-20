// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Elevator;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.motorcontrol.PWMTalonFX;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.simulation.PWMSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.WoodbotConstants;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.math.controller.ProfiledPIDController;

public class ElevatorIOSim implements ElevatorIO {
  /** Creates a new ElevatorIOSim. */

  private DCMotor gearbox = DCMotor.getFalcon500(1);
  public static Encoder encoder = new Encoder(0, 1);

  final double UPPER_LIMIT = 40;
  final double LOWER_LIMIT = 5;

  final double kA = 0.0;
  final double kD = 0.35;
  final double kG = 0.65;
  final double kI = 0.2;
  final double kP = 0.55;
  final double kS = 0.05;
  final double kV = 0.0;

  private ProfiledPIDController pidController = new ProfiledPIDController(kP, kI, kD,
      new TrapezoidProfile.Constraints(600, 300));
  private ElevatorFeedforward feedforward = new ElevatorFeedforward(kS, kG, kV, kA);

  private final PWMTalonFX motor = new PWMTalonFX(1);

  public final ElevatorSim elevatorSim = new ElevatorSim(
      gearbox,
      1.0, // elevator gearing
      2.0, // carriage mass
      Units.inchesToMeters(2.0), // elevator drum radius
      LOWER_LIMIT, // min elevator height meters
      UPPER_LIMIT, // max elevator height meters
      true,
      0,
      0.01,
      0.0);

  // private DCMotorSim elevatorSim = new DCMotorSim(
  // LinearSystemId.createDCMotorSystem(DCMotor.getFalcon500(1), 0.004,
  // motorReduction),
  // DCMotor.getFalcon500(1));

  private final EncoderSim simEncoder = new EncoderSim(encoder);
  private final PWMSim simMotor = new PWMSim(motor);

  private final LoggedMechanism2d mech2d = new LoggedMechanism2d(40, 50, new Color8Bit(Color.kAquamarine));
  private final LoggedMechanismRoot2d mech2dRoot = mech2d.getRoot("elevator root", 10, 0);
  private final LoggedMechanismLigament2d elevatorMech2d = mech2dRoot.append(
      new LoggedMechanismLigament2d("elevator", elevatorSim.getPositionMeters(), 90, 5, new Color8Bit(Color.kCoral)));

  public ElevatorIOSim() {
    // distance per pulse = (distance per revolution) / (pulses per revolution)
    encoder.setDistancePerPulse(2.0 * Math.PI * (Units.inchesToMeters(2.0)) / 4096);

    SmartDashboard.putData("elevator sim", mech2d);
  }

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {
    elevatorSim.setInput(simMotor.getSpeed() * RobotController.getBatteryVoltage());
    elevatorSim.update(0.02);
    simEncoder.setDistance(elevatorSim.getPositionMeters());
    RoboRioSim.setVInVoltage(BatterySim.calculateDefaultBatteryLoadedVoltage(elevatorSim.getCurrentDrawAmps()));
    //elevatorMech2d.setLength(elevatorSim.getPositionMeters());
    elevatorMech2d.setLength(encoder.getDistance());

    Logger.recordOutput("elevator sim", mech2d);
    inputs.elevatorPosition = elevatorSim.getPositionMeters();
    inputs.elevatorVelocity = simMotor.getSpeed();
  }

  //   public void updateTelemetry() {
  //   // Update elevator visualization with position
  //   elevatorMech2d.setLength(encoder.getDistance());
  // }

  public void setElevatorPostion(double height) {
    pidController.setGoal(height);
    // simMotor.setPosition(height);

    double pidOutput = pidController.calculate(simEncoder.getDistance());
    double feedforwardOutput = feedforward.calculate(pidController.getSetpoint().velocity);
    motor.setVoltage(pidOutput + feedforwardOutput);
    // simMotor.setPosition(height);
  }

  public void setDutyCycle(double dutyCycle) {
    simMotor.setSpeed(dutyCycle);
  }

  @Override
  public double getPosition() {
    return simMotor.getPosition();
  }

  @Override
  public boolean getBottomSwitch() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'getBottomSwitch'");
  }
}
