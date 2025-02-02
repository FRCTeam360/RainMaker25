// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.hal.HALUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Constants.OldCompBotConstants;
import frc.robot.commands.AlignWithLimelight;
import frc.robot.commands.SnapDrivebaseToAngle;
import frc.robot.generated.OldCompBot;
import frc.robot.generated.WoodBotDriveTrain;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Catapult.Catapult;
import frc.robot.subsystems.CoralIntake.CoralIntake;
import frc.robot.subsystems.CoralShooter.CoralShooter;
import frc.robot.subsystems.Elevator.Elevator;
import frc.robot.subsystems.Elevator.ElevatorIO;
import frc.robot.subsystems.Elevator.ElevatorIOWB;
import frc.robot.subsystems.Vision.Vision;
import frc.robot.subsystems.Vision.VisionIOLimelight;

public class RobotContainer {
    private final SendableChooser<Command> autoChooser;
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    private Telemetry logger;

    private final CommandXboxController driverCont = new CommandXboxController(0);
    private final CommandXboxController operatorCont = new CommandXboxController(1);
    
    private CommandFactory commandFactory;

    public static CommandSwerveDrivetrain driveTrain;
    private Vision vision;
    private Catapult catapult;
    private CoralIntake coralIntake;
    private CoralShooter coralShooter;
    private Elevator elevator;




    private ShuffleboardTab diagnosticTab;

    private SnapDrivebaseToAngle snapDrivebaseToAngle;
    private AlignWithLimelight alignWithLimelight;

    public RobotContainer() {
        switch (Constants.getRobotType()) {
            case WOODBOT:
                //woodbot stuff
                vision = new Vision(new VisionIOLimelight(
                    Constants.VisionConstants.WOODBOT_LIMELIGHT_NAME, 
                    Constants.VisionConstants.WOODBOT_YAW_FUDGE_FACTOR,
                    Constants.VisionConstants.WOODBOT_PITCH_FUDGE_FACTOR));

                driveTrain = WoodBotDriveTrain.createDrivetrain();
                logger = new Telemetry(WoodBotDriveTrain.maxSpeed);

                elevator = new Elevator(new ElevatorIOWB());
                break;
            case OLD_COMP_BOT:

                vision =
                    new Vision(
                        new VisionIOLimelight(
                            Constants.OldCompBotConstants.OCB_LIMELIGHT_NAME,
                            Constants.OldCompBotConstants.OCB_YAW_FUDGE_FACTOR,
                            Constants.OldCompBotConstants.OCB_PITCH_FUDGE_FACTOR
                        )
                    );
                driveTrain = OldCompBot.createDrivetrain();
                logger = new Telemetry(OldCompBot.maxSpeed);
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
                driveTrain = WoodBotDriveTrain.createDrivetrain();
                logger = new Telemetry(WoodBotDriveTrain.maxSpeed);
                break;
            case COMPETITION:
            default:
                //competition bot stuff
                break;

        }
        commandFactory = new CommandFactory(catapult, coralIntake, coralShooter, elevator, vision);

        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Auto Chooser", autoChooser);

        diagnosticTab = Shuffleboard.getTab("Diagnostics");
        diagnosticTab.addBoolean("Wood Bot", Constants::isWoodBot);
        diagnosticTab.addBoolean("Comp Bot", Constants::isCompBot);
        diagnosticTab.addBoolean("Practice Bot", Constants::isPracticeBot);
        diagnosticTab.addBoolean("Old Comp Bot", Constants::isOCB);
        diagnosticTab.addString("Serial Address", HALUtil::getSerialNumber);

        //initializeCommands();
        //configureBindings();
    }

    public void initializeCommands() {
        snapDrivebaseToAngle =
            new SnapDrivebaseToAngle(driveTrain);
        alignWithLimelight =
            new AlignWithLimelight(
                vision,
                driveTrain,
                0.0,
                3.0
            );
    }

    private void configureBindings() {
        driveTrain.setDefaultCommand(
            driveTrain.fieldOrientedDrive(MaxAngularRate, driverCont)
        );

        driverCont.pov(90).onTrue(new InstantCommand(() -> driveTrain.zero(), driveTrain));

        driverCont.a().onTrue(alignWithLimelight);
        driverCont.b().onTrue(snapDrivebaseToAngle);

        driveTrain.registerTelemetry(logger::telemeterize);
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}