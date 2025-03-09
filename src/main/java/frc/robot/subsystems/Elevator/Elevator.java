// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Elevator;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.Collections;
import java.util.Map;
import java.util.Set;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import java.util.stream.Collectors;

import org.littletonrobotics.junction.Logger;

public class Elevator extends SubsystemBase {
    private final ElevatorIO io;
    private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();

    private static final InterpolatingDoubleTreeMap offsetMap = new InterpolatingDoubleTreeMap();

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
                            return Math.abs(
                                    inputs.elevatorPosition - getElevatorHeightWithOffset(height, tySupplier)) <= 0.5;
                        })
                        .deadlineFor(
                                this.runEnd(
                                        () -> io.setElevatorPostion(getElevatorHeightWithOffset(height, tySupplier)),
                                        () -> io.setElevatorPostion(getElevatorHeightWithOffset(height, tySupplier)))));
    }

    private enum elevatorStates {
        L1,
        L2,
        L3,
        L4
    }

    private elevatorStates elevatorState = getElevatorState();

    private final Map<elevatorStates, Command> ELEVATOR_HEIGHTS = Map.ofEntries(
            Map.entry(elevatorStates.L1,
                    this.setElevatorHeight(Constants.SetPointConstants.ElevatorHeights.TELE_LEVEL_ONE)),
            Map.entry(elevatorStates.L2,
                    this.setElevatorHeight(Constants.SetPointConstants.ElevatorHeights.TELE_LEVEL_TWO)),
            Map.entry(elevatorStates.L3,
                    this.setElevatorHeight(Constants.SetPointConstants.ElevatorHeights.TELE_LEVEL_THREE)),
            Map.entry(elevatorStates.L4,
                    this.setElevatorHeight(Constants.SetPointConstants.ElevatorHeights.TELE_LEVEL_FOUR)));

    private Command raiseElevatorBasedOnDistance(Supplier<Pose2d> robotPose) {
        final Set<Integer> REEF_TAG_IDS = Set.of(6, 7, 8, 9, 10, 11, 17, 18, 19, 20, 21, 22);

        final Map<Integer, Pose2d> TAG_TO_POSITIONS = REEF_TAG_IDS.stream()
                .map(tagID -> Map.entry(tagID, Constants.FIELD_LAYOUT.getTagPose(tagID).get().toPose2d()))
                .collect(Collectors.toMap(
                        Map.Entry::getKey,
                        Map.Entry::getValue));

        Pose2d closestTag = TAG_TO_POSITIONS.get(0);
        double minDist = Double.MAX_VALUE;
        

        //trying to go through all the tag positions and set the closest tag to the one thats closest
            double distance = robotPose.getTranslation().getDistance(tag.pose.toPose2d().getTranslation());
            if (distance < minDist) {
                closestTag = tag;
                minDist = distance;
            }
        }

    }

    public Command setElevatorHeightWithStates() {
        return Commands.select(ELEVATOR_HEIGHTS, () -> this.elevatorState);
    }

    private elevatorStates getElevatorState() {
        return elevatorState.L1;
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

    public void zeroEncoder() {
        io.setEncoder(0.0);
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
