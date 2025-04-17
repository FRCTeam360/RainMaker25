// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.Utils;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.Constants.CompBotConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.AlgaeRoller.AlgaeRoller;
import frc.robot.subsystems.AlgaeShooter.AlgaeShooter;
import frc.robot.subsystems.AlgaeTilt.AlgaeTilt;
import frc.robot.subsystems.Vision.Vision;
import edu.wpi.first.math.MathUtil;


/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class VisionShootAlgae extends Command {
    private Vision vision;
    private AlgaeTilt algaeTilt;
    private AlgaeRoller algaeRoller;
    private AlgaeShooter algaeShooter;
    private CommandSwerveDrivetrain drivetrain;
    private CommandXboxController driverCont;

    private String LIMELIGHT_NAME = Constants.PracticeBotConstants.ALGAE_LIMELIGHT_NAME;
    private double setpoint;
    private double tolerance;
    private double angle;
    private double ty;
    private double heading;
    private boolean isFinished;

    InterpolatingDoubleTreeMap distanceVelocity = new InterpolatingDoubleTreeMap();

    InterpolatingDoubleTreeMap distanceAngle = new InterpolatingDoubleTreeMap();

    /** Creates a new VisionShootAlgae. */
    public VisionShootAlgae(CommandSwerveDrivetrain drivetrain, Vision vision, AlgaeRoller algaeRoller, AlgaeShooter algaeShooter, AlgaeTilt algaeTilt, CommandXboxController driverCont) {
        this.vision = vision;
        this.algaeRoller = algaeRoller;
        this.algaeShooter = algaeShooter;
        this.algaeTilt = algaeTilt;
        this.driverCont = driverCont;
        this.drivetrain = drivetrain;

        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(algaeRoller, algaeShooter, algaeTilt);
        
        // distanceVelocity.put(2.91, 5000.0);
        // distanceVelocity.put(9.14, 5250.0);
        // distanceVelocity.put(14.13, 5500.0);
        // distanceVelocity.put(18.2, .0);

        // distanceAngle.put(2.91, 0.035);
        // distanceAngle.put(9.14, 0.05);
        // distanceAngle.put(14.13, 0.055);
        // distanceAngle.put(18.2, 0.06);

        setpoint = 0.0;
        angle = 0.0;
        heading = 0.0;
        tolerance = 50;
        ty = 0.0;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {}

    public double angleCalculator(double ty) {
        double angle = 0.0000228102 * (Math.pow(ty, 3)) + (0.0000216486 * Math.pow(ty, 2)) + (0.000556059 * ty) + 0.0358254;
        angle = MathUtil.clamp(angle, 0.02, 0.06);

        return angle;
    }

    public double setpointCalculator(double ty) {
        double setpoint = (0.129115 * Math.pow(ty, 3)) + (1.67316 * Math.pow(ty, 2)) + (4641.44221 + 25.0);
        setpoint = MathUtil.clamp(setpoint, 4600.0, 5600.0);

        return setpoint;
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        ty = vision.getTYRaw(LIMELIGHT_NAME);
        setpoint = setpointCalculator(ty);
        angle = angleCalculator(ty);

        algaeShooter.setVelocity(setpoint);
        algaeTilt.setPosition(angle);

        if(drivetrain.getAngle() >= 0.5 && drivetrain.getAngle() <= 179.5) {
            heading = 90.0;
        } else if (drivetrain.getAngle() >= -179.5 && drivetrain.getAngle() <= -0.5) {
            heading = -90.0;
        }

        double xVel = -MathUtil.applyDeadband(Math.pow(driverCont.getLeftY(), 3), 0.1);
        double yVel = -MathUtil.applyDeadband(Math.pow(driverCont.getLeftX(), 3), 0.1);

        drivetrain.bargeFieldCentricFacingAngle(xVel, yVel, heading);

        if ((Math.abs(algaeShooter.getVelocity() - setpoint) < tolerance) && (Math.abs(xVel) <= 0.1) && (vision.getTV(LIMELIGHT_NAME) == 1)) {
            algaeRoller.setDutyCycle(1.0);
        }
        

        Logger.recordOutput("Algae RPM", setpoint);
        Logger.recordOutput("Algae Angle",angle);
        Logger.recordOutput("Algae TY", vision.getTYRaw(LIMELIGHT_NAME));
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
