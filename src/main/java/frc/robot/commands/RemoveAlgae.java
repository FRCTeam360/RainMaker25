// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.Constants.*;
import frc.robot.Constants.SetPointConstants.ElevatorHeights;
import frc.robot.subsystems.AlgaeArm.AlgaeArm;
import frc.robot.subsystems.AlgaeRoller.AlgaeRoller;
import frc.robot.subsystems.AlgaeShooter.AlgaeShooter;
import frc.robot.subsystems.AlgaeTilt.AlgaeTilt;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.CoralShooter.CoralShooter;
import frc.robot.subsystems.Elevator.Elevator;
import frc.robot.subsystems.Vision.Vision;

import java.util.HashMap;
import java.util.Map;

import org.littletonrobotics.junction.Logger;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class RemoveAlgae extends Command {
    private AlgaeArm algaeArm;
    private AlgaeShooter algaeShooter;
    private AlgaeRoller algaeRoller;
    private AlgaeTilt algaeTilt;
    private CoralShooter coralShooter;
    private Elevator elevator;
    private Vision vision;

    private double height = SetPointConstants.ElevatorHeights.TELE_LEVEL_FOUR;

    private int id;

 private static final Map<Integer, String> removeHeight = Map.ofEntries(
        Map.entry(6, "low"),
        Map.entry(8, "low"),
        Map.entry(10, "low"),
        Map.entry(17, "low"),
        Map.entry(21, "low"),
        Map.entry(19, "low"),
        Map.entry(7, "high"),
        Map.entry(9, "high"),
        Map.entry(11, "high"),
        Map.entry(18, "high"),
        Map.entry(20, "high"),
        Map.entry(22, "high")
    );

    /** Creates a new RemoveAlgae. */
    public RemoveAlgae(
        AlgaeArm algaeArm,
        AlgaeShooter algaeShooter,
        AlgaeRoller algaeRoller,
        AlgaeTilt algaeTilt,
        CoralShooter coralShooter,
        Elevator elevator,
        Vision vision
    ) {
        this.algaeArm = algaeArm;
        this.algaeShooter = algaeShooter;
        this.algaeRoller = algaeRoller;
        this.algaeTilt = algaeTilt;
        this.coralShooter = coralShooter;
        this.elevator = elevator;
        this.vision = vision;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(algaeArm, algaeShooter, algaeRoller, algaeTilt, elevator);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        id = vision.getAprilTagID(Constants.CompBotConstants.CORAL_LIMELIGHT_NAME);
        height = elevator.getHeight();

        if (removeHeight.get(id) == "high") {
            height = ElevatorHeights.TELE_LEVEL_THREE;
        } else if (removeHeight.get(id) == "low") {
            height = ElevatorHeights.TELE_LEVEL_TWO;
        } else if(height >= 11.0) {
            height = ElevatorHeights.TELE_LEVEL_THREE;
        } else if (height < 11.0) {
            height = ElevatorHeights.TELE_LEVEL_TWO;
        }
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        coralShooter.setDutyCycle(-1.0);
        algaeTilt.setPosition(0.12);
        algaeShooter.setDutyCycle(-1.0);
        algaeRoller.setDutyCycle(-0.5);
        elevator.setElevatorPostion(height + 6.5);
        algaeArm.setPosition(22.0);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        coralShooter.stop();
        algaeShooter.stop();
        algaeRoller.stop();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
