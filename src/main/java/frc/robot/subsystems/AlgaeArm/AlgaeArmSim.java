// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.AlgaeArm;

import java.util.DoubleSummaryStatistics;
import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Unit;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.simulation.PWMSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.CoralShooter.CoralShooterIO.CoralShooterIOInputs;

public class AlgaeArmSim implements AlgaeArmIO {
  
  private DoubleSupplier heightSupplier;
  private DCMotor gearbox = DCMotor.getNeo550(1);
  private Encoder encoder = new Encoder(2, 3);

  private final PWMSparkMax motor = new PWMSparkMax(2);
  private final PWMSim simMotor = new PWMSim(motor);

  private final SingleJointedArmSim armSim = new SingleJointedArmSim(gearbox, 1.0, SingleJointedArmSim.estimateMOI(Units.inchesToMeters(30) /*placeholder*/ , 8.0/*placeist*/),Units.inchesToMeters(30)/*placeholder*/ , Units.degreesToRadians(-75)/*placehold */, Units.degreesToRadians(255)/*place */, true, 0 , (2.0 * Math.PI/4096));
 
  private final EncoderSim encoderSim = new EncoderSim(encoder);
 
  private final LoggedMechanism2d mech2d = new LoggedMechanism2d(50, 50, new Color8Bit(Color.kPapayaWhip));
  private final LoggedMechanismRoot2d mechRoot2D = mech2d.getRoot("algea arm root", 0, 0);
  private final LoggedMechanismLigament2d mechLigiment2D = mechRoot2D.append(new LoggedMechanismLigament2d("algea arm",0.3 , 0,0.05, new Color8Bit(Color.kTomato)));
  
  /** Creates a new AlgaeArmSim. */
  public AlgaeArmSim(DoubleSupplier heightSupplier) {
    this.heightSupplier = heightSupplier;
    encoderSim.setDistancePerPulse(2.0 * Math.PI/4096);
  }

  public void updateInputs(AlgaeArmIOInputs inputs) {
    armSim.setInput(simMotor.getSpeed() * RobotController.getBatteryVoltage());
    armSim.update(0.02);
    encoderSim.setDistance(armSim.getAngleRads());
    RoboRioSim.setVInVoltage(BatterySim.calculateDefaultBatteryLoadedVoltage(armSim.getCurrentDrawAmps()));
    inputs.algaeArmPosition = simMotor.getPosition();
    inputs.algaeArmStatorCurrent = armSim.getCurrentDrawAmps();
    inputs.algaeArmVelocity = armSim.getVelocityRadPerSec();

    mechLigiment2D.setAngle(new Rotation2d(armSim.getAngleRads()));
    mechRoot2D.setPosition(0, heightSupplier.getAsDouble());
    Logger.recordOutput("elevter sim", mech2d);
  }

  @Override
  public void setDutyCycle(double dutyCycle) {
    simMotor.setSpeed(dutyCycle);
  }

}
