// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.Constants.*;
import frc.robot.subsystems.AlgaeRoller.AlgaeRoller;
import frc.robot.subsystems.AlgaeShooter.AlgaeShooter;
import frc.robot.subsystems.AlgaeTilt.AlgaeTilt;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Vision.Vision;
import java.util.Map;
import java.util.Optional;
import org.littletonrobotics.junction.Logger;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class BargeAlign extends Command {
    private final CommandSwerveDrivetrain driveTrain;
    private final CommandXboxController driverCont;
    private final AlgaeShooter algaeShooter;
    private final AlgaeTilt algaeTilt;
    private final AlgaeRoller algaeRoller;
    private final Vision vision;

    private double angle;
    private double goalTY = 0.5;
    private int id;

    private final String CMD_NAME = "Barge Align: ";

    private static final Map<Integer, Double> tagIDToAngle = Map.ofEntries( //blue pov
        Map.entry(13, -90.0),
        Map.entry(14, -90.0),
        Map.entry(4, 90.0),
        Map.entry(5, 90.0)
    );

    /** Creates a new BargeAlign. */
    public BargeAlign(
        CommandSwerveDrivetrain driveTrain,
        Vision vision,
        AlgaeShooter algaeShooter,
        AlgaeTilt algaeTilt,
        AlgaeRoller algaeRoller,
        CommandXboxController driverCont
    ) {
        this.driveTrain = driveTrain;
        this.vision = vision;
        this.algaeShooter = algaeShooter;
        this.driverCont = driverCont;
        this.algaeTilt = algaeTilt;
        this.algaeRoller = algaeRoller;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(driveTrain, algaeShooter, algaeTilt, algaeRoller);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        id = vision.getAprilTagID(CompBotConstants.ALGAE_LIMELIGHT_NAME);
        angle = tagIDToAngle.get(id);
        Optional<Alliance> alliance = DriverStation.getAlliance();
        if (alliance.isPresent() && alliance.get() == Alliance.Red) {
            angle += 180.0;
        }

        Logger.recordOutput(CMD_NAME + "angle", angle);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        // return Commands.waitUntil(() -> algaeShooter.getVelocity() > 5750)
        // .andThen(algaeRoller.setDutyCycleCmd(1.0))
        // .alongWith(algaeShooter.setVelocityCmd(6250))
        // .alongWith(algaeTilt.setPositionCmd(Constants.isCompBot() ? 0.03 : 3.0));

        algaeShooter.setVelocity(6250.0);
        algaeTilt.setPosition(Constants.isCompBot() ? 0.03 : 3.0);

        double velX = driveTrain.forwardController.calculate(
            vision.getTYRaw(CompBotConstants.ALGAE_LIMELIGHT_NAME),
            goalTY,
            driveTrain.getState().Timestamp
        );

        Logger.recordOutput(CMD_NAME + "velX", velX);

        driveTrain.driveFieldCentricFacingAngle(velX, driverCont.getLeftX(), angle);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        algaeRoller.setDutyCycle(1.0);
        Logger.recordOutput(CMD_NAME + )
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        //shooter is at vel angle is good ty is good
        boolean onTY = driveTrain.forwardController.atSetpoint();
        boolean atAngle = driveTrain.headingController.atSetpoint();
        boolean atVelocity = algaeShooter.getVelocity() > 5750;

        Logger.recordOutput(CMD_NAME + "onTY", onTY);
        Logger.recordOutput(CMD_NAME + "atAngle", atAngle);
        Logger.recordOutput(CMD_NAME + "atVelocity", atVelocity);

        return onTY && atAngle && atVelocity;
    }
}
