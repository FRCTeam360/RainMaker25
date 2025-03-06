// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Elevator;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

public class Elevator extends SubsystemBase {
    private final ElevatorIO io;
    private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();

    private static final InterpolatingDoubleTreeMap offsetMap = new InterpolatingDoubleTreeMap();

    private final LoggedMechanism2d mech2d = new LoggedMechanism2d(20, 50, new Color8Bit(Color.kBlue));
    private final LoggedMechanismRoot2d mech2dRoot = mech2d.getRoot("elevator root", 10, 0);
    private final LoggedMechanismLigament2d elevatorMech2d = mech2dRoot.append(new LoggedMechanismLigament2d("elevator", inputs.elevatorPosition, 90, 5, new Color8Bit(Color.kCoral)));

    /** Creates a new Elevator. */
    public Elevator(ElevatorIO io) {
        this.io = io;
        offsetMap.put(6.0, 23.0 + 0.0);
        offsetMap.put(7.0, 23.0 + 0.2);
        offsetMap.put(8.0, 23.0 + 0.4);
    }

    /**
     * 
     * @param height is in rotations
     */
    public void setElevatorPostion(double height) {
        io.setElevatorPostion(height);
    }

    public void setDutyCycle(double dutyCycle) {
        io.setDutyCycle(dutyCycle);
    }

    public boolean getBottomSwitch() {
        return inputs.elevatorSensor;
    }

    public double getHeight() {
        return inputs.elevatorPosition;
    }

    public void stop() {
        io.stop();
    }

    public double getElevatorHeightWithOffset(double height, DoubleSupplier tySupplier) {
        return offsetMap.get(tySupplier.getAsDouble()) + height;
    }

    public Command setElevatorHeightDynamic(double height, DoubleSupplier tySupplier) {
        return Commands
                .waitUntil(() -> tySupplier.getAsDouble() < 0.0) // TODO: replace
                    .andThen(Commands 
                    .waitUntil(() -> {
                    return Math.abs(inputs.elevatorPosition - getElevatorHeightWithOffset(height, tySupplier)) <= 0.5;
                })
                .deadlineFor(
                        this.runEnd(
                                () -> io.setElevatorPostion(getElevatorHeightWithOffset(height, tySupplier)),
                                () -> io.setElevatorPostion(getElevatorHeightWithOffset(height, tySupplier)))));
    }

    /**
     * @param height is in rotations
     * @return
     */
    public Command setElevatorHeight(double height) {
        return Commands
                .waitUntil(() -> Math.abs(inputs.elevatorPosition) <= 0.5)
                .deadlineFor(
                        this.runEnd(
                                () -> io.setElevatorPostion(height),
                                () -> io.setElevatorPostion(height)));
    }

    public Command setDutyCycleCommand(DoubleSupplier duty) {
        return this.runEnd(
                () -> io.setDutyCycle(duty.getAsDouble()),
                () -> io.setDutyCycle(0.0));
    }

    public Command zeroElevatorCmd() {
        return Commands
                .waitUntil(
                        () -> Math.abs(inputs.elevatorStatorCurrent) > 30.0 &&
                                Math.abs(inputs.elevatorVelocity) == 0.0)
                .deadlineFor(this.runEnd(() -> io.setDutyCycle(-0.08), () -> io.setDutyCycle(0.0)))
                .andThen(this.runOnce(() -> io.setEncoder(0.0)));
    }

    public void zeroEncoder(){
        io.setEncoder(0.0);
    }

    public Command isAtHeight(double position) {
        return Commands.waitUntil(() -> Math.abs(inputs.elevatorPosition - position) <= 1.0);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        elevatorMech2d.setLength(inputs.elevatorPosition);
        io.updateInputs(inputs);
        Logger.recordOutput("Elevator Mech", mech2d);
        Logger.processInputs("Elevator", inputs);
    }
}
