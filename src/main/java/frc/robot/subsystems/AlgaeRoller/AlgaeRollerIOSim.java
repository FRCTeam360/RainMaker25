// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.AlgaeRoller;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkFlex;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.WoodbotConstants;
import frc.robot.subsystems.Elevator.ElevatorIOSim;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.PWMSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;

public class AlgaeRollerIOSim implements AlgaeRollerIO {
  private DoubleSupplier heightSupplier;

  private DCMotor gearbox = DCMotor.getNEO(1);
  private Encoder encoder = new Encoder(Constants.SimulationConstants.SHOOTER_ENCODER_A, Constants.SimulationConstants.SHOOTER_ENCODER_B);

  private final PWMSparkMax motor = new PWMSparkMax(Constants.SimulationConstants.CORAL_SHOOTER_ID);

  private final LinearSystem<N1, N1, N1> plant =
    LinearSystemId.createFlywheelSystem(
      gearbox,0.00113951385, 1.0); // TODO: find actual MOI

  private final FlywheelSim flywheelSim = new FlywheelSim(
    plant, // 
    gearbox, // gearbox
    0.01);

  private final EncoderSim simEncoder = new EncoderSim(encoder);
  private final PWMSim simMotor = new PWMSim(motor);
  
  private final Color8Bit color = new Color8Bit(Color.kCoral);
  private final LoggedMechanism2d mech2d = new LoggedMechanism2d(30, 50, new Color8Bit(Color.kBlue));
  private final LoggedMechanismRoot2d mech2dRoot = mech2d.getRoot("shooter root", 10, 0);

  /** Creates a new AlgaeIntakeRollerIOPB. */
  public AlgaeRollerIOSim(DoubleSupplier heightSupplier) {
    this.heightSupplier = heightSupplier;
  }

  public void setDutyCycle(double dutyCycle) {
    simMotor.setSpeed(dutyCycle);
  }

  public void updateInputs(AlgaeRollerIOInputs inputs) {
    simEncoder.setDistancePerPulse(2.0 * Math.PI * (Units.inchesToMeters(2.0)) / 4096);
    flywheelSim.setInput(simMotor.getSpeed() * RobotController.getBatteryVoltage());
    flywheelSim.update(0.02);
    simEncoder.setDistance(simMotor.getPosition());
    RoboRioSim.setVInVoltage(BatterySim.calculateDefaultBatteryLoadedVoltage(flywheelSim.getCurrentDrawAmps()));
    mech2dRoot.setPosition(10, (heightSupplier.getAsDouble() + 5.0));

    Logger.recordOutput("elevator sim", mech2d);
    SmartDashboard.putData("shooter sim", mech2d);
    inputs.rollerPosition = simMotor.getPosition();
    inputs.rollerVelocity = flywheelSim.getAngularVelocityRPM();
  }
}
