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
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Constants.OldCompBotConstants;
import frc.robot.commands.AlignWithLimelight;
import frc.robot.commands.SetCoralIntake;
import frc.robot.commands.SnapDrivebaseToAngle;
import frc.robot.generated.OldCompBot;
import frc.robot.generated.WoodBotDriveTrain;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Elevator.Elevator;
import frc.robot.subsystems.Elevator.ElevatorIOSim;
import frc.robot.subsystems.Catapult.Catapult;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.CoralIntake.CoralIntake;
import frc.robot.subsystems.CoralShooter.CoralShooter;
import frc.robot.subsystems.CoralShooter.CoralShooterIOWB;
import frc.robot.subsystems.CoralShooter.CoralShooterIOSim;
import frc.robot.subsystems.Elevator.Elevator;
import frc.robot.subsystems.Elevator.ElevatorIO;
import frc.robot.subsystems.Elevator.ElevatorIOWB;
import frc.robot.subsystems.Vision.Vision;
import frc.robot.subsystems.Vision.VisionIO;
import frc.robot.subsystems.Vision.VisionIOLimelight;

public class RobotContainer {
    private final Field2d field;
    private final SendableChooser<Command> autoChooser;
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    private Telemetry logger;
    
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
    private Command alignWithLimelight;

    private Command levelFour;
    private Command levelThree;
    private Command levelTwo;
    private Command zero;

    private Command allignToReefWoodBot;

    private SetCoralIntake setCoralIntake;

    public RobotContainer() {
        switch (Constants.getRobotType()) {
            case WOODBOT:
                //woodbot stuff
                driveTrain = WoodBotDriveTrain.createDrivetrain();
                logger = new Telemetry(WoodBotDriveTrain.kSpeedAt12Volts.in(MetersPerSecond));
                vision = new Vision(new VisionIO[]{
                    new VisionIOLimelight(
                        Constants.VisionConstants.WOODBOT_LIMELIGHT_NAME,
                        Constants.VisionConstants.WOODBOT_YAW_FUDGE_FACTOR,
                        Constants.VisionConstants.WOODBOT_PITCH_FUDGE_FACTOR,
                        () -> driveTrain.getAngle()
                    )
                });
                elevator = new Elevator(new ElevatorIOWB());
                coralShooter = new CoralShooter(new CoralShooterIOWB());
                break;
            case OLD_COMP_BOT:
                //ocb stuff
                driveTrain = OldCompBot.createDrivetrain();
                vision = new Vision(new VisionIO[] {
                        new VisionIOLimelight(
                                Constants.OldCompBotConstants.OCB_LIMELIGHT_NAME,
                                Constants.OldCompBotConstants.OCB_YAW_FUDGE_FACTOR,
                                Constants.OldCompBotConstants.OCB_PITCH_FUDGE_FACTOR,
                                () -> driveTrain.getAngle()) });
                //constants = Constants.OldCompBotConstants;
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
                        () -> driveTrain.getAngle()
                    )
                });
                elevator = new Elevator(new ElevatorIOSim());
                coralShooter = new CoralShooter(new CoralShooterIOSim(() -> elevator.getHeight()));
                break;
            case COMPETITION:
            default:
                //competition bot stuff
                break;
        }
        commandFactory = new CommandFactory(catapult, coralIntake, coralShooter, elevator, vision, driveTrain, driverCont.getHID());
        initializeCommands();

        field = new Field2d();
        SmartDashboard.putData("Field", field);

        PathPlannerLogging.setLogActivePathCallback(
        (poses -> Logger.recordOutput("Swerve/ActivePath", poses.toArray(new Pose2d[0]))));
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

        initializeCommands();
        configureBindings();
    }

    public void initializeCommands() {


        alignWithLimelight =
            commandFactory.AlignWithLimelight(Constants.WoodbotConstants.WBGOALSCORETY, 
            Constants.WoodbotConstants.WBGOALSCORETX);

        snapDrivebaseToAngle =
            new SnapDrivebaseToAngle(driveTrain);
        // alignWithLimelight =
        //     new AlignWithLimelight(
        //         vision,
        //         driveTrain,
        //         0.0,
        //         3.0,
        //         0.25,
        //         Constants.OldCompBotConstants.maxAngularRate
        //     );

        if(Objects.nonNull(elevator)){
            levelFour = commandFactory.setElevatorHeight(34.0);
            levelThree = commandFactory.setElevatorHeight(18.0);
            levelTwo = commandFactory.setElevatorHeight(8.5);
            zero = commandFactory.setElevatorHeight(0.0);

            
            NamedCommands.registerCommand(
                "raise to l4",
                commandFactory.setElevatorHeight(33.0).raceWith(elevator.isAtHeight(33.0))
            );
            NamedCommands.registerCommand(
                "zero",
                commandFactory.setElevatorHeight(0.0).raceWith(elevator.isAtHeight(0.0))
            );
        }

        allignToReefWoodBot = commandFactory.allignToReefWoodbotLeft();

        if(Objects.nonNull(coralShooter)){
         
            setCoralIntake = new SetCoralIntake(coralShooter);
            
            NamedCommands.registerCommand("shoot", coralShooter.shootCmd());
            
            NamedCommands.registerCommand("intake", coralShooter.intakeCmd());
        }
    }


    private void configureBindings() {
        driveTrain.setDefaultCommand(
            driveTrain.fieldOrientedDrive(MaxAngularRate, driverCont)
        );

        driverCont.pov(0).onTrue(new InstantCommand(() -> driveTrain.zero(), driveTrain));
       // driverCont.pov(180).onTrue(allignToReefWoodBot);
        driverCont.pov(90).onTrue(snapDrivebaseToAngle);

        if(Objects.nonNull(elevator)){
            driverCont.a().onTrue(zero);
            driverCont.b().whileTrue(alignWithLimelight);
            driverCont.x().onTrue(levelThree);
            driverCont.y().onTrue(levelFour);
        }

        if(Objects.nonNull(coralShooter)){
            driverCont.leftBumper().whileTrue(setCoralIntake);
            driverCont.rightBumper().whileTrue(coralShooter.shootCmd());
        }
        //driverCont.rightBumper().whileTrue(new InstantCommand(() -> coralShooter.setDutyCycle(-0.3), coralShooter)); //add end T-T

        //driveTrain.registerTelemetry(logger::telemeterize);
    }

    public void onDisable() {
        if(Objects.nonNull(elevator)){
            elevator.stop();
        }
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}
