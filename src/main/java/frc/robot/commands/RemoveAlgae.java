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
    private AlgaeTilt algaeTilt;
    private CoralShooter coralShooter;
    private Elevator elevator;
    private Vision vision;

    private double height = SetPointConstants.ElevatorHeights.TELE_LEVEL_FOUR;

    private int id;

 private static final Map<Integer, String> removeHeight = Map.ofEntries(
        Map.entry(6, "high"),
        Map.entry(8, "high"),
        Map.entry(10, "high"),
        Map.entry(17, "high"),
        Map.entry(21, "high"),
        Map.entry(19, "high"),
        Map.entry(7, "low"),
        Map.entry(9, "low"),
        Map.entry(11, "low"),
        Map.entry(18, "low"),
        Map.entry(20, "low"),
        Map.entry(22, "low")
    );

    /** Creates a new RemoveAlgae. */
    public RemoveAlgae(
        AlgaeArm algaeArm,
        AlgaeShooter algaeShooter,
        AlgaeTilt algaeTilt,
        CoralShooter coralShooter,
        Elevator elevator,
        Vision vision
    ) {
        this.algaeArm = algaeArm;
        this.algaeShooter = algaeShooter;
        this.algaeTilt = algaeTilt;
        this.coralShooter = coralShooter;
        this.elevator = elevator;
        this.vision = vision;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(algaeArm, algaeShooter, algaeTilt, elevator);
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
        algaeTilt.setPosition(0.0);
        algaeShooter.setDutyCycle(-0.8);

        algaeArm.setPosition(100.0);
        elevator.setElevatorPostion(height + 1.0);
        // if (coralShooter.getVelocity() < -6000.0) {
        // }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        coralShooter.stop();
        algaeShooter.stop();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
