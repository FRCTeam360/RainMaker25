// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Elevator;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Elevator extends SubsystemBase {
    private final ElevatorIO io;
    private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();

    /** Creates a new Elevator. */
    public Elevator(ElevatorIO io) {
        this.io = io;
    }

    public void setElevatorPostion(double height) {
        io.setElevatorPostion(height);
    }

    public void setDutyCycle(double dutyCycle) {
        io.setDutyCycle(dutyCycle);
    }

    public boolean getBottomSwitch() {
        return io.getBottomSwitch();
    }

    public Command setElevatorHeight(double height) {
        return this.runEnd(
                () -> io.setElevatorPostion(height),
                () -> io.setElevatorPostion(height)
            );
    }

    public void stop() {
        io.stop();
    }

    public Command isAtHeight(double position) {
        return Commands.waitUntil(() -> Math.abs(inputs.elevatorPosition - position) <= 1.0);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        io.updateInputs(inputs);
        Logger.processInputs("Elevator", inputs);
    }
}
