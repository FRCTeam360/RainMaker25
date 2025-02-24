package frc.robot.subsystems.AlgaeShooter;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AlgaeShooter extends SubsystemBase {
    private final AlgaeShooterIO io;
    private final AlgaeShooterIOInputsAutoLogged inputs = new AlgaeShooterIOInputsAutoLogged();

    // Creates AlgaeShooter
    public AlgaeShooter(AlgaeShooterIO io) {
        this.io = io;
    }

    public void setDutyCycle(double dutyCycle) {
        io.setDutyCycle(dutyCycle);
    }

    public void setVelocity(double velocity) {
        io.setDutyCycle(velocity);
    }

    public void stop() {
        io.stop();
    }

    public double getVelocity() {
        return inputs.algaeShooterFrontVelocity;
    }

    public Command setDutyCycleCmd(double dutyCycle) {
        return this.runEnd(() -> this.setDutyCycle(dutyCycle), () -> this.stop());
    }

    public Command setVelocityCmd(double velocity) {
        return this.runEnd(() -> this.setVelocity(velocity), () -> this.setVelocity(0));
    }

    public Command waitVelocitySetpoint(double velocitySetpoint) {
        return Commands.waitUntil(() -> Math.abs(velocitySetpoint - inputs.algaeShooterFrontVelocity) < 100);
    }
    
    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Algae Shooter", inputs);
    }
}