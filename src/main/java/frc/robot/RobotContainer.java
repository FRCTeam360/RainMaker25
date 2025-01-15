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
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Constants.OldCompBotConstants;
import frc.robot.commands.AlignWithLimelight;
import frc.robot.commands.SnapDrivebaseToAngle;
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

    public static CommandSwerveDrivetrain driveTrain;
    private Vision vision;

    private ShuffleboardTab diagnosticTab;

    private AlignWithLimelight alignWithLimelight;
    private SnapDrivebaseToAngle snapDrivebaseToAngle;

    private Class<?> constants;

    public RobotContainer() {
        switch (Constants.getRobotType()) {
            case OLD_COMP_BOT:
                //ocb stuff
                constants = Constants.getOldCompBotConstants();
                vision =
                    new Vision(
                        new VisionIOLimelight(
                            Constants.OldCompBotConstants.OCB_LIMELIGHT_NAME,
                            Constants.OldCompBotConstants.OCB_YAW_FUDGE_FACTOR,
                            Constants.OldCompBotConstants.OCB_PITCH_FUDGE_FACTOR
                        )
                    );
                driveTrain = OldCompBot.createDrivetrain();
                //constants = Constants.OldCompBotConstants;
                setUpDrivetrain(
                    vision,
                    Constants.OldCompBotConstants.headingKP,
                    Constants.OldCompBotConstants.headingKI,
                    Constants.OldCompBotConstants.headingKD,
                    Constants.OldCompBotConstants.translationKP,
                    Constants.OldCompBotConstants.translationKI,
                    Constants.OldCompBotConstants.translationKD
                );
            case WOODBOT:
                //woodbot stuff
                vision =
                    new Vision(
                        new VisionIOLimelight(
                            Constants.VisionConstants.WOODBOT_LIMELIGHT_NAME,
                            Constants.VisionConstants.WOODBOT_YAW_FUDGE_FACTOR,
                            Constants.VisionConstants.WOODBOT_PITCH_FUDGE_FACTOR
                        )
                    );
                driveTrain = OldCompBot.createDrivetrain();
                break;
            case PRACTICE:
                //practice bot stuff
                break;
            case SIM:
                vision =
                    new Vision(
                        new VisionIOLimelight(
                            Constants.OldCompBotConstants.OCB_LIMELIGHT_NAME,
                            Constants.OldCompBotConstants.OCB_YAW_FUDGE_FACTOR,
                            Constants.OldCompBotConstants.OCB_PITCH_FUDGE_FACTOR
                        )
                    );
                driveTrain = OldCompBot.createDrivetrain();
                //constants = Constants.OldCompBotConstants;
                setUpDrivetrain(
                    vision,
                    Constants.OldCompBotConstants.headingKP,
                    Constants.OldCompBotConstants.headingKI,
                    Constants.OldCompBotConstants.headingKD,
                    Constants.OldCompBotConstants.translationKP,
                    Constants.OldCompBotConstants.translationKI,
                    Constants.OldCompBotConstants.translationKD
                );
                break;
            case COMPETITION:
            default:
                //competition bot stuff
                break;
        }

        diagnosticTab = Shuffleboard.getTab("Diagnostics");
        diagnosticTab.addBoolean("Wood Bot", Constants::isWoodBot);
        diagnosticTab.addBoolean("Comp Bot", Constants::isCompBot);
        diagnosticTab.addBoolean("Practice Bot", Constants::isPracticeBot);
        diagnosticTab.addBoolean("Old Comp Bot", Constants::isOCB);
        diagnosticTab.addString("Serial Address", HALUtil::getSerialNumber);

        initializeCommands();
        configureBindings();
    }

    public void initializeCommands() {
        snapDrivebaseToAngle = new SnapDrivebaseToAngle(driveTrain, MaxSpeed);
        alignWithLimelight =
            new AlignWithLimelight(
                vision,
                driveTrain,
                25.0,
                Constants.OldCompBotConstants.maxSpeed,
                Constants.OldCompBotConstants.maxAngularRate
            );
    }

    private static void setUpDrivetrain(
        Vision vision,
        double headingKP,
        double headingKI,
        double headingKD,
        double translationKP,
        double translationKI,
        double translationKD
    ) {
        driveTrain.addHeadingController(headingKP, headingKI, headingKD);
        driveTrain.addTranslationController(translationKP, translationKI, translationKD);
        driveTrain.assignVision(vision);
    }

    private void configureBindings() {
        driveTrain.setDefaultCommand(
            driveTrain.fieldOrientedDrive(MaxSpeed, MaxAngularRate, driverCont)
        );

        driverCont.pov(90).onTrue(new InstantCommand(() -> driveTrain.zero(), driveTrain));

        driverCont.b().onTrue(snapDrivebaseToAngle);
        driverCont.a().onTrue(new SequentialCommandGroup(snapDrivebaseToAngle, alignWithLimelight));

        driveTrain.registerTelemetry(logger::telemeterize);
    }

    public Command getAutonomousCommand() {
        return Commands.print("No autonomous command configured");
    }
}
