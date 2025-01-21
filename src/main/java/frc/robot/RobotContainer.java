// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
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
import frc.robot.commands.SetCoralIntake;
import frc.robot.commands.SnapDrivebaseToAngle;
import frc.robot.generated.OldCompBot;
import frc.robot.generated.WoodBotDriveTrain;
import frc.robot.subsystems.Catapult.Catapult;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.CoralIntake.CoralIntake;
import frc.robot.subsystems.CoralShooter.CoralShooter;
import frc.robot.subsystems.CoralShooter.CoralShooterIOWB;
import frc.robot.subsystems.Elevator.Elevator;
import frc.robot.subsystems.Elevator.ElevatorIO;
import frc.robot.subsystems.Elevator.ElevatorIOWB;
import frc.robot.subsystems.Vision.Vision;
import frc.robot.subsystems.Vision.VisionIOLimelight;

public class RobotContainer {
    private final SendableChooser<Command> autoChooser;
    private double MaxSpeed = OldCompBot.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController driverCont = new CommandXboxController(0);
    private final CommandXboxController operatorCont = new CommandXboxController(1);

    private CommandFactory commandFactory;

    private CommandSwerveDrivetrain driveTrain;
    private Vision vision;
    private Catapult catapult;
    private CoralIntake coralIntake;
    private CoralShooter coralShooter;
    private Elevator elevator;

    private ShuffleboardTab diagnosticTab;

    private SnapDrivebaseToAngle snapDrivebaseToAngle;
    private AlignWithLimelight alignWithLimelight;

    private Command levelFour;
    private Command levelThree;
    private Command levelTwo;
    private Command zero;

    private SetCoralIntake setCoralIntake;

    public RobotContainer() {
        switch (Constants.getRobotType()) {
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
                driveTrain = WoodBotDriveTrain.createDrivetrain();
                setUpDrivetrain(
                    vision,
                    Constants.OldCompBotConstants.headingKP,
                    Constants.OldCompBotConstants.headingKI,
                    Constants.OldCompBotConstants.headingKD,
                    Constants.OldCompBotConstants.headingKIZone,
                    Constants.OldCompBotConstants.translationKP,
                    Constants.OldCompBotConstants.translationKI,
                    Constants.OldCompBotConstants.translationKD
                );
                elevator = new Elevator(new ElevatorIOWB());
                coralShooter = new CoralShooter(new CoralShooterIOWB());
                break;
            case OLD_COMP_BOT:
                //ocb stuff

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
                    Constants.OldCompBotConstants.headingKIZone,
                    Constants.OldCompBotConstants.translationKP,
                    Constants.OldCompBotConstants.translationKI,
                    Constants.OldCompBotConstants.translationKD
                );
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
                    Constants.OldCompBotConstants.headingKIZone,
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
        commandFactory = new CommandFactory(catapult, coralIntake, coralShooter, elevator, vision);
        initializeCommands();

        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Auto Chooser", autoChooser);

        diagnosticTab = Shuffleboard.getTab("Diagnostics");
        diagnosticTab.addBoolean("Wood Bot", Constants::isWoodBot);
        diagnosticTab.addBoolean("Comp Bot", Constants::isCompBot);
        diagnosticTab.addBoolean("Practice Bot", Constants::isPracticeBot);
        diagnosticTab.addBoolean("Old Comp Bot", Constants::isOCB);
        diagnosticTab.addString("Serial Address", HALUtil::getSerialNumber);

        configureBindings();
    }

    public void initializeCommands() {
        // snapDrivebaseToAngle =
        //     new SnapDrivebaseToAngle(driveTrain, Constants.OldCompBotConstants.maxSpeed);
        // alignWithLimelight =
        //     new AlignWithLimelight(
        //         vision,
        //         driveTrain,
        //         0.0,
        //         3.0,
        //         0.25,
        //         Constants.OldCompBotConstants.maxAngularRate
        //     );

        levelFour = commandFactory.setElevatorHeight(34.0);
        levelThree = commandFactory.setElevatorHeight(25.0);
        levelTwo = commandFactory.setElevatorHeight(10.0);
        zero = commandFactory.setElevatorHeight(0.0);

        setCoralIntake = new SetCoralIntake(coralShooter);

        NamedCommands.registerCommand("raise to l4", commandFactory.setElevatorHeight(34.0).raceWith(elevator.isAtHeight(34.0)));
        NamedCommands.registerCommand("zero", commandFactory.setElevatorHeight(0.0).raceWith(elevator.isAtHeight(0.0)));

        NamedCommands.registerCommand(
            "shoot",
            coralShooter.shootCmd()
        );
    }

    private void setUpDrivetrain(
        Vision vision,
        double headingKP,
        double headingKI,
        double headingKD,
        double headingKIZone,
        double translationKP,
        double translationKI,
        double translationKD
    ) {
        driveTrain.addHeadingController(headingKP, headingKI, headingKD, headingKIZone);
        driveTrain.addTranslationController(translationKP, translationKI, translationKD);
        driveTrain.assignVision(vision);
    }

    private void configureBindings() {
        driveTrain.setDefaultCommand(
            driveTrain.fieldOrientedDrive(MaxSpeed, MaxAngularRate, driverCont)
        );

        driverCont.pov(0).onTrue(new InstantCommand(() -> driveTrain.zero(), driveTrain));

        driverCont.a().onTrue(zero);
        driverCont.b().onTrue(levelTwo);
        driverCont.x().onTrue(levelThree);
        driverCont.y().onTrue(levelFour);

        driverCont.leftBumper().whileTrue(setCoralIntake);
        driverCont
            .rightBumper()
            .whileTrue(
                coralShooter.shootCmd()
            ); //add end T-T
        //driverCont.rightBumper().whileTrue(new InstantCommand(() -> coralShooter.setDutyCycle(-0.3), coralShooter)); //add end T-T

        //driveTrain.registerTelemetry(logger::telemeterize);
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}
