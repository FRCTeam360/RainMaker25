// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.SignalLogger;
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
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Constants.OldCompBotConstants;
import frc.robot.commands.AlignWithLimelight;
import frc.robot.commands.SetCoralIntake;
import frc.robot.commands.SnapDrivebaseToAngle;
import frc.robot.generated.OldCompBot;
import frc.robot.generated.PracticeBotDriveTrain;
import frc.robot.generated.WoodBotDriveTrain;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.ClimberWinch.ClimberWinch;
import frc.robot.subsystems.ClimberWinch.ClimberWinchIOPB;
import frc.robot.subsystems.ClimberWinch.ClimberWinchIOSim;
import frc.robot.subsystems.AlgaeShooter.AlgaeShooter;
import frc.robot.subsystems.AlgaeShooter.AlgaeShooterIOPB;
import frc.robot.subsystems.AlgaeShooter.AlgaeShooterIOSim;
import frc.robot.subsystems.AlgaeTilt.AlgaeTilt;
import frc.robot.subsystems.AlgaeTilt.AlgaeTiltIOPB;
import frc.robot.subsystems.ClimberWheel.ClimberWheel;
import frc.robot.subsystems.ClimberWheel.ClimberWheelIOPB;
import frc.robot.subsystems.ClimberWheel.ClimberWheelIOSim;
import frc.robot.subsystems.Elevator.Elevator;
import frc.robot.subsystems.Elevator.ElevatorIOSim;
import frc.robot.subsystems.CoralShooter.CoralShooter;
import frc.robot.subsystems.CoralShooter.CoralShooterIOPB;
import frc.robot.subsystems.CoralShooter.CoralShooterIOSim;
import frc.robot.subsystems.CoralShooter.CoralShooterIOWB;
import frc.robot.subsystems.Elevator.ElevatorIO;
import frc.robot.subsystems.Elevator.ElevatorIOPB;
import frc.robot.subsystems.Elevator.ElevatorIOSim;
import frc.robot.subsystems.Elevator.ElevatorIOSim;
import frc.robot.subsystems.Elevator.ElevatorIOWB;
import frc.robot.subsystems.Vision.Vision;
import frc.robot.subsystems.Vision.VisionIO;
import frc.robot.subsystems.Vision.VisionIOLimelight;
import frc.robot.subsystems.AlgaeArm.AlgaeArm;
import frc.robot.subsystems.AlgaeArm.AlgaeArmIOPB;
import frc.robot.subsystems.AlgaeArm.AlgaeArmIOSim;
import frc.robot.subsystems.AlgaeRoller.AlgaeRoller;
import frc.robot.subsystems.AlgaeRoller.AlgaeRollerIO;
import frc.robot.subsystems.AlgaeRoller.AlgaeRollerIOPB;

import java.lang.ModuleLayer.Controller;
import java.util.HashMap;
import java.util.Map;
import java.util.Objects;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class RobotContainer {
    private final Field2d field;
    private final SendableChooser<Command> autoChooser;
    // max angular velocity

    private Telemetry logger;

    private final CommandXboxController driverCont = new CommandXboxController(0);
    private final CommandXboxController operatorCont = new CommandXboxController(1);
    private final CommandXboxController testCont = new CommandXboxController(5);

    private CommandFactory commandFactory;
    private Command setAngle;
    private CommandSwerveDrivetrain driveTrain;
    private Vision vision;
    private CoralShooter coralShooter;
    private Elevator elevator;
    private ClimberWinch climberWinch;
    private ClimberWheel climberWheel;
    private AlgaeArm algaeArm;
    private AlgaeShooter algaeShooter;
    private AlgaeRoller algaeRoller;
    private AlgaeTilt algaeTilt;

    private ShuffleboardTab diagnosticTab;

    private Command rightAlign;
    private Command leftAlign;
    private SnapDrivebaseToAngle snapDrivebaseToAngle;

    private Command levelFour;
    private Command levelThree;
    private Command levelTwo;
    private Command levelOne;

    private Command autoLevelFour;
    private Command autoLevelThree;

    private Command scoreLevel3RightTeleop;

    private Command zeroElevatorEncoder;

    private SequentialCommandGroup levelOneAndZero;

    private Command robotCentricDrive;

    private Command xOut;
    private Command setSteerCoast;

    private Command allignToReefWoodBot;
    private SetCoralIntake setCoralIntake;

    private Command scoreCoralL4Right = null;
    private Command scoreCoralL3Left = null;
    private Command scoreCoralL4Left = null;
    private Command scoreCoralL3Right = null;
    private Command scoreCoralL2Left = null;
    private Command scoreCoralL2Right = null;
    private Command scoreCoralL1 = null;

    private Command consumeVisionMeasurements;

    public RobotContainer() {
        switch (Constants.getRobotType()) {
            case WOODBOT:
                driveTrain = WoodBotDriveTrain.createDrivetrain();
                logger = new Telemetry(WoodBotDriveTrain.kSpeedAt12Volts.in(MetersPerSecond));
                vision =
                    new Vision(
                        Map.ofEntries(
                            Map.entry(
                                Constants.PracticeBotConstants.CORAL_LIMELIGHT_NAME,
                                new VisionIOLimelight(
                                    Constants.PracticeBotConstants.CORAL_LIMELIGHT_NAME,
                                    () -> driveTrain.getAngle(),
                                    () -> driveTrain.getAngularRate()
                                )
                            )
                        )
                    );
                elevator = new Elevator(new ElevatorIOPB());
                coralShooter = new CoralShooter(new CoralShooterIOWB());
                break;
            case OLD_COMP_BOT:
                driveTrain = OldCompBot.createDrivetrain();
                vision =
                    new Vision(
                        Map.ofEntries(
                            Map.entry(
                                Constants.PracticeBotConstants.CORAL_LIMELIGHT_NAME,
                                new VisionIOLimelight(
                                    Constants.PracticeBotConstants.CORAL_LIMELIGHT_NAME,
                                    () -> driveTrain.getAngle(),
                                    () -> driveTrain.getAngularRate()
                                )
                            )
                            // Map.entry(
                            //     Constants.PracticeBotConstants.ALGAE_LIMELIGHT_NAME,
                            //     new VisionIOLimelight(
                            //         Constants.PracticeBotConstants.ALGAE_LIMELIGHT_NAME,
                            //         () -> driveTrain.getAngle(),
                            //         () -> driveTrain.getAngularRate()
                            //     )
                            // )
                        )
                    );
                break;
            case PRACTICE:
                driveTrain = PracticeBotDriveTrain.createDrivetrain();
                logger = new Telemetry(PracticeBotDriveTrain.kSpeedAt12Volts.in(MetersPerSecond));
                coralShooter = new CoralShooter(new CoralShooterIOPB());
                elevator = new Elevator(new ElevatorIOPB());
                algaeArm = new AlgaeArm(new AlgaeArmIOPB());
                algaeRoller = new AlgaeRoller(new AlgaeRollerIOPB());
                algaeShooter = new AlgaeShooter(new AlgaeShooterIOPB());
                algaeTilt = new AlgaeTilt(new AlgaeTiltIOPB());
                climberWheel = new ClimberWheel(new ClimberWheelIOPB());
                climberWinch = new ClimberWinch(new ClimberWinchIOPB());

                vision =
                    new Vision(
                        Map.ofEntries(
                            Map.entry(
                                Constants.PracticeBotConstants.CORAL_LIMELIGHT_NAME,
                                new VisionIOLimelight(
                                    Constants.PracticeBotConstants.CORAL_LIMELIGHT_NAME,
                                    () -> driveTrain.getAngle(),
                                    () -> driveTrain.getAngularRate()
                                )
                            )
                            // Map.entry(
                            //     Constants.PracticeBotConstants.ALGAE_LIMELIGHT_NAME,
                            //     new VisionIOLimelight(
                            //         Constants.PracticeBotConstants.ALGAE_LIMELIGHT_NAME,
                            //         () -> driveTrain.getAngle(),
                            //         () -> driveTrain.getAngularRate()
                            //     )
                            // )
                        )
                    );
                // practice bot stuff
                break;
            case SIM:
                driveTrain = WoodBotDriveTrain.createDrivetrain();
                logger = new Telemetry(WoodBotDriveTrain.kSpeedAt12Volts.in(MetersPerSecond));
                vision =
                    new Vision(
                        Map.ofEntries(
                            Map.entry(
                                Constants.PracticeBotConstants.CORAL_LIMELIGHT_NAME,
                                new VisionIOLimelight(
                                    Constants.PracticeBotConstants.CORAL_LIMELIGHT_NAME,
                                    () -> driveTrain.getAngle(),
                                    () -> driveTrain.getAngularRate()
                                )
                            )
                            // Map.entry(
                            //     Constants.PracticeBotConstants.ALGAE_LIMELIGHT_NAME,
                            //     new VisionIOLimelight(
                            //         Constants.PracticeBotConstants.ALGAE_LIMELIGHT_NAME,
                            //         () -> driveTrain.getAngle(),
                            //         () -> driveTrain.getAngularRate()
                            //     )
                            // )
                        )
                    );
                elevator = new Elevator(new ElevatorIOSim());
                algaeArm = new AlgaeArm(new AlgaeArmIOSim(() -> elevator.getHeight()));
                coralShooter = new CoralShooter(new CoralShooterIOSim(() -> elevator.getHeight()));
                climberWinch = new ClimberWinch(new ClimberWinchIOSim());
                climberWheel = new ClimberWheel(new ClimberWheelIOSim());
                algaeShooter = new AlgaeShooter(new AlgaeShooterIOSim());
                break;
            case COMPETITION:
            default:
                // competition bot stuff
                break;
        }

        commandFactory =
            new CommandFactory(
                coralShooter,
                elevator,
                vision,
                climberWinch,
                climberWheel,
                algaeShooter,
                algaeArm,
                driveTrain,
                driverCont.getHID(),
                algaeTilt,
                algaeRoller
            );

        initializeCommands();

        field = new Field2d();
        SmartDashboard.putData("Field", field);

        PathPlannerLogging.setLogActivePathCallback(
            (poses -> Logger.recordOutput("Swerve/ActivePath", poses.toArray(new Pose2d[0])))
        );
        PathPlannerLogging.setLogTargetPoseCallback(
        pose -> Logger.recordOutput("Swerve/TargetPathPose", pose));
        
        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Auto Chooser", autoChooser);

        diagnosticTab = Shuffleboard.getTab("Diagnostics");
        diagnosticTab.addBoolean("Wood Bot", Constants::isWoodBot);
        diagnosticTab.addBoolean("Comp Bot", Constants::isCompBot);
        diagnosticTab.addBoolean("Practice Bot", Constants::isPracticeBot);
        diagnosticTab.addBoolean("Old Comp Bot", Constants::isOCB);
        diagnosticTab.addString("Serial Address", HALUtil::getSerialNumber);
        diagnosticTab.addBoolean("Sim", Constants::isSim);

        configureBindings();
        // configureTestController();
    }

    public void initializeCommands() {
        consumeVisionMeasurements = vision.consumeVisionMeasurements(driveTrain::addVisionMeasurements);

        xOut = driveTrain.xOutCmd();

        rightAlign =
            commandFactory.alignWithLimelight(
                Constants.WoodbotConstants.WBGOALSCORETY,
                Constants.WoodbotConstants.WBGOALSCORETX,
                0
            );
        // Periodically adds the vision measurement to drivetrain for pose estimation

        leftAlign =
            commandFactory.alignWithLimelight(
                Constants.WoodbotConstants.WBGOALSCORETY,
                Constants.WoodbotConstants.WBGOALSCORETX,
                1
            );

        snapDrivebaseToAngle = new SnapDrivebaseToAngle(vision, driveTrain, 0);

        if (Objects.nonNull(elevator)) {
            levelFour = commandFactory.setElevatorLevelFour();
            levelThree = commandFactory.setElevatorLevelThree();
            levelTwo = commandFactory.setElevatorLevelTwo();
            levelOne = commandFactory.setElevatorToZero();

            autoLevelThree = commandFactory.setElevatorHeight(20.5);
            autoLevelFour = commandFactory.setElevatorHeight(33.0);

            zeroElevatorEncoder = elevator.zeroElevatorCmd();

            levelOneAndZero = new SequentialCommandGroup(levelOne, zeroElevatorEncoder);

            NamedCommands.registerCommand(
                "raise to l4",
                commandFactory.setElevatorHeight(33.0).raceWith(elevator.isAtHeight(33.0))
            );
            NamedCommands.registerCommand(
                "zero",
                commandFactory.setElevatorHeight(0.0).raceWith(elevator.isAtHeight(0.0))
            );
        }

        if (Objects.nonNull(driveTrain)) {
            rightAlign =
                commandFactory.alignWithLimelight(
                    Constants.WoodbotConstants.WBGOALSCORETY,
                    Constants.WoodbotConstants.WBGOALSCORETX,
                    0
                );

            leftAlign =
                commandFactory.alignWithLimelight(
                    Constants.WoodbotConstants.WBGOALSCORETY,
                    Constants.WoodbotConstants.WBGOALSCORETX,
                    1
                );
        }

        registerPathplannerCommand("Elevator L4", autoLevelFour);
        registerPathplannerCommand("Elevator L3", autoLevelThree);
        registerPathplannerCommand("Elevator L2", levelTwo);
        registerPathplannerCommand("Elevator L1", levelOne);


        Command scoreCoralL4Left = null;
        Command scoreCoralL4Right = null;
        Command scoreCoralL3Left = null;
        Command scoreCoralL3Right = null;
        Command scoreCoralL2Left = null;
        Command scoreCoralL2Right = null;
        Command scoreCoralL1 = null;
        
        allignToReefWoodBot = commandFactory.alignToReefWoodbotLeft(0);       

        if (Objects.nonNull(coralShooter) && Objects.nonNull(elevator)) {
            scoreCoralL4Left = commandFactory.scoringRoutine(4, true);
            scoreCoralL4Right = commandFactory.scoringRoutine(4, false);
            scoreCoralL3Left = commandFactory.scoringRoutine(3, true);
            scoreCoralL3Right = commandFactory.scoringRoutine(3, false);
            scoreCoralL2Left = commandFactory.scoringRoutine(2, true);
            scoreCoralL2Right = commandFactory.scoringRoutine(2, false);
            scoreCoralL1 = commandFactory.scoreLevelOne();

            scoreLevel3RightTeleop = commandFactory.scoringRoutineTeleop(3, false);
        }

        registerPathplannerCommand("Score Coral L4 Left", scoreCoralL4Left);
        registerPathplannerCommand("Score Coral L4 Right", scoreCoralL4Right);
        registerPathplannerCommand("Score Coral L3 Left", scoreCoralL3Left);
        registerPathplannerCommand("Score Coral L3 Right", scoreCoralL3Right);
        registerPathplannerCommand("Score Coral L2 Left", scoreCoralL2Left);
        registerPathplannerCommand("Score Coral L2 Right", scoreCoralL2Right);
        registerPathplannerCommand("Score Coral L1", scoreCoralL1);

        registerPathplannerCommand("left align", leftAlign);

        registerPathplannerCommand("x out", xOut);

        registerPathplannerCommand("consume vision measurements", consumeVisionMeasurements);

        Command intake = null;

        if (Objects.nonNull(coralShooter)) {
            setCoralIntake = new SetCoralIntake(coralShooter);
            intake = coralShooter.basicIntakeCmd();

            NamedCommands.registerCommand("shoot", coralShooter.basicShootCmd());
            NamedCommands.registerCommand("intake", coralShooter.basicIntakeCmd());
        }

        registerPathplannerCommand("Intake Coral", intake);
    }

    /**
     * This method registers the given command to as a named pathplanner command if
     * the command is present. If not, it registers in its place a placeholder
     * command that outputs a warning to the console
     *
     * @param string  the registered name of the command as shown in the pathplanner
     *                auto/path file
     * @param command the actual command
     */
    private void registerPathplannerCommand(String commandName, Command command) {
        if (Objects.nonNull(command)) {
            NamedCommands.registerCommand(commandName, command);
        } else {
            System.err.println(commandName + " is null");
            NamedCommands.registerCommand(
                commandName,
                new InstantCommand(() -> System.err.println(commandName + " is null"))
            );
        }
    }

    private void configureBindings() {
        // elevator.setDefaultCommand(elevator.setDutyCycleCommand(() -> operatorCont.getLeftY() * 0.05));
        driveTrain.setDefaultCommand(driveTrain.fieldOrientedDrive(driverCont));
        algaeTilt.setDefaultCommand(algaeTilt.setDutyCycleCmd(() -> operatorCont.getLeftY() * 0.10));
        algaeArm.setDefaultCommand(algaeArm.setAlgaeArmAngleCmd(0.0));

        operatorCont.pov(0).whileTrue(new InstantCommand(() -> algaeTilt.setEncoder(-10.0), algaeTilt));
        operatorCont.pov(90).whileTrue(commandFactory.outtakeAlgaeFromGround());
        operatorCont.pov(180).whileTrue(commandFactory.shootAlgae());
        operatorCont.pov(270).whileTrue(commandFactory.intakeAlgaeFromGround());
        
        operatorCont.y().whileTrue(algaeTilt.setPositionCmd(0.0));
        operatorCont.x().whileTrue(algaeTilt.setPositionCmd(5.0));
        operatorCont.b().whileTrue(algaeTilt.setPositionCmd(30.0));
        operatorCont.a().whileTrue(algaeTilt.setPositionCmd(35.0));

        operatorCont.leftBumper().whileTrue(algaeRoller.setDutyCycleCmd(-0.8));
        operatorCont.rightBumper().whileTrue(algaeRoller.setDutyCycleCmd(0.8));

        driverCont.rightStick().whileTrue(driveTrain.robotCentricDrive(driverCont));

        driverCont.pov(0).onTrue(new InstantCommand(() -> driveTrain.zero(), driveTrain));

        driverCont.leftTrigger(0.25).whileTrue(coralShooter.sensorIntakeCmd()); 
        driverCont.rightTrigger(0.25).whileTrue(coralShooter.basicShootCmd());

        if (Objects.nonNull(elevator)) {
            driverCont.a().onTrue(levelOneAndZero);
            driverCont.b().onTrue(levelTwo);
            driverCont.x().onTrue(levelThree);
            driverCont.y().whileTrue(levelFour);

            driverCont.pov(270).onTrue(commandFactory.removeAlgaeL3());
            driverCont.pov(90).onTrue(commandFactory.removeAlgaeL2());
            driverCont.pov(180).onTrue(commandFactory.retractAlgaeArm());
        }

        if (Objects.nonNull(coralShooter)) {
            driverCont.leftBumper().whileTrue(leftAlign);
            driverCont.rightBumper().whileTrue(rightAlign);
        }

        // driverCont.leftBumper().onTrue(Commands.runOnce(SignalLogger::start));
        // driverCont.rightBumper().onTrue(Commands.runOnce(SignalLogger::stop));
        /*
         * Joystick Y = quasistatic forward
         * Joystick A = quasistatic reverse
         * Joystick X = dyanmic reverse
         * Joystick B = dynamic forward
         */

        // driverCont.y().whileTrue(driveTrain.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
        // driverCont.a().whileTrue(driveTrain.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
        // driverCont.b().whileTrue(driveTrain.sysIdDynamic(SysIdRoutine.Direction.kForward));
        // driverCont.x().whileTrue(driveTrain.sysIdDynamic(SysIdRoutine.Direction.kReverse));
    }

    private void configureTestController() {
        // climberWinch.setDefaultCommand(climberWinch.setDutyCycleCmd(() -> testCont.getLeftY()));
        algaeArm.setDefaultCommand(algaeArm.setDutyCycleCmd(() -> testCont.getLeftY()));

        // Algae commands`
        // testCont.a().whileTrue(commandFactory.intakeAlgaeFromGround());
        // testCont.b().whileTrue(commandFactory.outtakeAlgaeFromGround());
        // testCont.y().whileTrue(commandFactory.shootAlgae());
        // testCont.x().whileTrue(commandFactory.intakeAlgaeFromReef());
        // testCont.a().whileTrue(algaeArm.setAlgaeArmAngleCmd(110.0));
        testCont.a().whileTrue(commandFactory.setElevatorHeightZeroAndZero());
        testCont.b().whileTrue(commandFactory.setElevatorLevelTwo());
        testCont.x().whileTrue(commandFactory.setElevatorLevelThree());
        testCont.y().whileTrue(commandFactory.setElevatorLevelFour());

        testCont.rightBumper().whileTrue(commandFactory.removeAlgaeL3());
        testCont.rightTrigger(0.5).whileTrue(commandFactory.removeAlgaeL2());
        testCont.start().whileTrue(commandFactory.retractAlgaeArm());

        testCont.leftBumper().whileTrue(commandFactory.intakeAlgaeFromReef());
        testCont.pov(0).whileTrue(algaeTilt.setPositionCmd(0));

        // testCont.b().whileTrue(algaeArm.setAlgaeArmAngleCmd(0));

        // elevator commands
    }

    public void onDisable() {
        if (Objects.nonNull(elevator)) {
            elevator.stop();
        }
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected().beforeStarting(() -> elevator.zeroEncoder());
    }
}
