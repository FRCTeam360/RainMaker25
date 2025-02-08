// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import java.lang.ModuleLayer.Controller;
import java.util.Objects;
import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.util.PathPlannerLogging;

import edu.wpi.first.hal.HALUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
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
import frc.robot.subsystems.Elevator.Elevator;
import frc.robot.subsystems.Elevator.ElevatorIOSim;
import frc.robot.subsystems.Catapult.Catapult;
import frc.robot.subsystems.CoralIntake.CoralIntake;
import frc.robot.subsystems.CoralShooter.CoralShooter;
import frc.robot.subsystems.CoralShooter.CoralShooterIOSim;
import frc.robot.subsystems.Elevator.Elevator;
import frc.robot.subsystems.Elevator.ElevatorIO;
import frc.robot.subsystems.Elevator.ElevatorIOWB;
import frc.robot.subsystems.Vision.Vision;
import frc.robot.subsystems.Vision.VisionIO;
import frc.robot.subsystems.Vision.VisionIOLimelight;
import frc.robot.subsystems.AlgaeArm.AlgaeArm;
import frc.robot.subsystems.AlgaeArm.AlgaeArmIOSim;

public class RobotContainer {
    private final Field2d field;
    private final SendableChooser<Command> autoChooser;
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    private Telemetry logger;

    private CommandFactory commandFactory;

    private final CommandXboxController driverCont = new CommandXboxController(0);
    private final CommandXboxController operatorCont = new CommandXboxController(1);

    private Command levelFour;
    private Command levelThree;
    private Command levelTwo;
    private Command zero;

    public static CommandSwerveDrivetrain driveTrain;
    private Vision vision;
    private Catapult catapult;
    private CoralIntake coralIntake;
    private CoralShooter coralShooter;
    private Elevator elevator;
    private AlgaeArm algaeArm;

    private ShuffleboardTab diagnosticTab;

    private SnapDrivebaseToAngle snapDrivebaseToAngle;
    private AlignWithLimelight alignWithLimelight;

    public RobotContainer() {
        switch (Constants.getRobotType()) {
            case WOODBOT:
                //woodbot stuff
                driveTrain = WoodBotDriveTrain.createDrivetrain();
                logger = new Telemetry(WoodBotDriveTrain.kSpeedAt12Volts.in(MetersPerSecond));
                vision = new Vision(new VisionIO[]{
                    new VisionIOLimelight(
                        Constants.OldCompBotConstants.OCB_LIMELIGHT_NAME,
                        Constants.OldCompBotConstants.OCB_YAW_FUDGE_FACTOR,
                        Constants.OldCompBotConstants.OCB_PITCH_FUDGE_FACTOR,
                        () -> driveTrain.getAngle(),
                        () -> driveTrain.getAngularRate()
                    )
                });
                elevator = new Elevator(new ElevatorIOWB());
                break;
            case OLD_COMP_BOT:
                //ocb stuff
                driveTrain = OldCompBot.createDrivetrain();
                logger = new Telemetry(OldCompBot.kSpeedAt12Volts.in(MetersPerSecond));
                vision = new Vision(new VisionIO[]{
                            new VisionIOLimelight(
                                Constants.OldCompBotConstants.OCB_LIMELIGHT_NAME,
                                Constants.OldCompBotConstants.OCB_YAW_FUDGE_FACTOR,
                                Constants.OldCompBotConstants.OCB_PITCH_FUDGE_FACTOR,
                                () -> driveTrain.getAngle(),
                                () -> driveTrain.getAngularRate()
                            )
                        });
                break;
            case PRACTICE:
                //practice bot stuff
                break;
            case SIM:
                driveTrain = WoodBotDriveTrain.createDrivetrain();
                logger = new Telemetry(WoodBotDriveTrain.kSpeedAt12Volts.in(MetersPerSecond));
                vision = new Vision(new VisionIO[]{
                    new VisionIOLimelight(
                        Constants.OldCompBotConstants.OCB_LIMELIGHT_NAME,
                        Constants.OldCompBotConstants.OCB_YAW_FUDGE_FACTOR,
                        Constants.OldCompBotConstants.OCB_PITCH_FUDGE_FACTOR,
                        () -> driveTrain.getAngle(),
                        () -> driveTrain.getAngularRate()
                    )
                });
                elevator = new Elevator(new ElevatorIOSim());
                coralShooter = new CoralShooter(new CoralShooterIOSim(() -> elevator.getPosition()));
                algaeArm = new AlgaeArm(new AlgaeArmIOSim(() -> elevator.getPosition()));
                break;
            case COMPETITION:
            default:
                //competition bot stuff
                break;
        }

        field = new Field2d();
        SmartDashboard.putData("Field", field);

        PathPlannerLogging.setLogActivePathCallback(
        (poses -> Logger.recordOutput("Swerve/ActivePath", poses.toArray(new Pose2d[0]))));
        PathPlannerLogging.setLogTargetPoseCallback(
        pose -> Logger.recordOutput("Swerve/TargetPathPose", pose));
        
        commandFactory = new CommandFactory(catapult, coralIntake, coralShooter, elevator, vision, algaeArm);

        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Auto Chooser", autoChooser);

        diagnosticTab = Shuffleboard.getTab("Diagnostics");
        diagnosticTab.addBoolean("Wood Bot", Constants::isWoodBot);
        diagnosticTab.addBoolean("Comp Bot", Constants::isCompBot);
        diagnosticTab.addBoolean("Practice Bot", Constants::isPracticeBot);
        diagnosticTab.addBoolean("Old Comp Bot", Constants::isOCB);
        diagnosticTab.addString("Serial Address", HALUtil::getSerialNumber);
        diagnosticTab.addBoolean("Sim", Constants::isSim);

        initializeCommands();
        configureBindings();
    }

    public void initializeCommands() {
        // Periodically adds the vision measurement to drivetrain for pose estimation
        vision.setDefaultCommand(
                vision.consumeVisionMeasurements(driveTrain::addVisionMeasurements).ignoringDisable(true)
        );

        snapDrivebaseToAngle =
            new SnapDrivebaseToAngle(driveTrain);
        alignWithLimelight =
            new AlignWithLimelight(
                vision,
                driveTrain,
                0.0,
                3.0
            );

        if(Objects.nonNull(elevator)){
            levelFour = commandFactory.setElevatorHeight(34.0);
            levelThree = commandFactory.setElevatorHeight(25.0);
            levelTwo = commandFactory.setElevatorHeight(10.0);
            zero = commandFactory.setElevatorHeight(0.0);
        }
    }

    /**
     * This method registers the given command to as a named pathplanner command if the command is present. If not, it registers in its place a placeholder command that outputs a warning to the console
     * @param string the registered name of the command as shown in the pathplanner auto/path file
     * @param command the actual command
     */
    private void registerPathplannerCommand(String commandName, Command command) {
        if (Objects.nonNull(command)) {
            NamedCommands.registerCommand(commandName, command);
        } else {
            System.err.println(commandName + " is null");
            NamedCommands.registerCommand(commandName,
                    new InstantCommand(() -> System.err.println(commandName + " is null")));
        }
    }

    private void configureBindings() {
        driveTrain.setDefaultCommand(
            driveTrain.fieldOrientedDrive(MaxAngularRate, driverCont)
        );

        driverCont.pov(90).onTrue(new InstantCommand(() -> driveTrain.zero(), driveTrain));

        // driverCont.a().onTrue(alignWithLimelight);
        // driverCont.b().onTrue(snapDrivebaseToAngle);

        driveTrain.registerTelemetry(logger::telemeterize);

        if(Objects.nonNull(elevator)){
            driverCont.a().onTrue(zero);
            driverCont.b().onTrue(levelTwo);
            driverCont.x().onTrue(levelThree);
            driverCont.y().onTrue(levelFour);
        }
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}