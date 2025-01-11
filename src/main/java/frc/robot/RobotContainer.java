// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.hal.HALUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

import frc.robot.generated.OldCompBot;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Vision.Vision;
import frc.robot.subsystems.Vision.VisionIOLimelight;

public class RobotContainer {
    private double MaxSpeed = OldCompBot.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    private final Telemetry logger = new Telemetry(MaxSpeed);


    private final CommandXboxController driverCont = new CommandXboxController(0);
    private final CommandXboxController operatorCont = new CommandXboxController(1);

    public CommandSwerveDrivetrain driveTrain;
    private Vision vision;

    private ShuffleboardTab diagnosticTab;


    public RobotContainer() {
        switch (Constants.getRobotType()) {
            case WOODBOT:
                //woodbot stuff
                vision = new Vision(new VisionIOLimelight(
                    Constants.VisionConstants.WOODBOT_LIMELIGHT_NAME, 
                    Constants.VisionConstants.WOODBOT_YAW_FUDGE_FACTOR,
                    Constants.VisionConstants.WOODBOT_PITCH_FUDGE_FACTOR));
                driveTrain = OldCompBot.createDrivetrain();
                break;
            case OLD_COMP_BOT:
                //ocb stuff
                vision = new Vision(new VisionIOLimelight(
                    Constants.VisionConstants.OCB_LIMELIGHT_NAME,
                    Constants.VisionConstants.OCB_YAW_FUDGE_FACTOR,
                    Constants.VisionConstants.OCB_PITCH_FUDGE_FACTOR));
                driveTrain = OldCompBot.createDrivetrain();
            case PRACTICE:
                //practice bot stuff
                break;
            case COMPETITION:
                //competition bot stuff
                break;
            default:
                break;
        }

        diagnosticTab = Shuffleboard.getTab("Diagnostics");
        diagnosticTab.addBoolean("Wood Bot",Constants::isWoodBot);
        diagnosticTab.addBoolean("Comp Bot", Constants::isCompBot);
        diagnosticTab.addBoolean("Practice Bot", Constants::isPracticeBot);
        diagnosticTab.addBoolean("Old Comp Bot", Constants::isOCB);
        diagnosticTab.addString("Serial Address", HALUtil::getSerialNumber);


      configureBindings();
    }
    private void configureBindings() {
        driveTrain.setDefaultCommand(
           driveTrain.fieldOrientedDrive(MaxSpeed, MaxAngularRate, driverCont)
        );

        driveTrain.registerTelemetry(logger::telemeterize);
    }

    public Command getAutonomousCommand() {
        return Commands.print("No autonomous command configured");
    }
    
}
