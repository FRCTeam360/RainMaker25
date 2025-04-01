// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.Constants.CompBotConstants;
import frc.robot.subsystems.AlgaeRoller.AlgaeRoller;
import frc.robot.subsystems.AlgaeShooter.AlgaeShooter;
import frc.robot.subsystems.AlgaeTilt.AlgaeTilt;
import frc.robot.subsystems.Vision.Vision;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class VisionShootAlgae extends Command {
    private Vision vision;
    private AlgaeTilt algaeTilt;
    private AlgaeRoller algaeRoller;
    private AlgaeShooter algaeShooter;
    private String LIMELIGHT_NAME = Constants.PracticeBotConstants.ALGAE_LIMELIGHT_NAME;
    private double setpoint;
    private double tolerance;
    private double angle;
    private boolean isFinished;

    InterpolatingDoubleTreeMap distanceVelocity = new InterpolatingDoubleTreeMap();

    InterpolatingDoubleTreeMap distanceAngle = new InterpolatingDoubleTreeMap();

    /** Creates a new VisionShootAlgae. */
    public VisionShootAlgae(Vision vision, AlgaeRoller algaeRoller, AlgaeShooter algaeShooter, AlgaeTilt algaeTilt) {
        this.vision = vision;
        this.algaeRoller = algaeRoller;
        this.algaeShooter = algaeShooter;
        this.algaeTilt = algaeTilt;

        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(algaeRoller, algaeShooter, algaeTilt);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        distanceVelocity.put(2.91, 5000.0);
        distanceVelocity.put(14.13, 5500.0);
        distanceVelocity.put(9.14, 5250.0);

        distanceAngle.put(2.91, 0.035);
        distanceAngle.put(14.13, 0.055);
        distanceAngle.put(9.14, 0.05);

        setpoint = 0;
        angle = 0;
        tolerance = 50;
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        setpoint = distanceVelocity.get(vision.getTYRaw(LIMELIGHT_NAME));
        angle = distanceAngle.get(vision.getTYRaw(LIMELIGHT_NAME));

        algaeShooter.setVelocity(setpoint);
        algaeTilt.setPosition(angle);

        if (Math.abs(algaeShooter.getVelocity() - setpoint) < tolerance) {
            algaeRoller.setDutyCycle(1.0);
        }

        Logger.recordOutput("Algae RPM", setpoint);
        Logger.recordOutput("Algae Angle", distanceAngle.get(vision.getTYRaw(CompBotConstants.ALGAE_LIMELIGHT_NAME)));
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        algaeRoller.stop();
        algaeShooter.stop();
        algaeTilt.stop();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
