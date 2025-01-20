// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.CoralShooter;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CoralShooterIOSim implements CoralShooterIO {

  private DCMotor gearbox = DCMotor.getNEO(1);

  private final double KP = 0.0;
    private final double KI = 0.0;
    private final double KD = 0.0;
    private final double KF = 0.0;

  private ProfiledPIDController pidController = new ProfiledPIDController(KP, KI, KD, 
    new TrapezoidProfile.Constraints(2.45, 2.45));

  /** Creates a new CoralOuttakeIOSim. */
  public CoralShooterIOSim() {}

  public void updateInputs(CoralShooterIOInputs inputs) {

  }

  @Override
  public void setDutyCycle(double dutyCycle) {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'setDutyCycle'");
  }

  @Override
  public boolean getOuttakeSensor() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'getOuttakeSensor'");
  }

  @Override
  public void stop() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'stop'");
  }
}
