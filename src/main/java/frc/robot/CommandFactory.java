package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.networktables.PubSub;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SelectCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.*;
import frc.robot.Constants.SetPointConstants.ElevatorHeights;
import frc.robot.commands.AlignWithLimelight;
import frc.robot.commands.PIDToReefPoints;
import frc.robot.commands.SetCoralIntake;
import frc.robot.commands.SmartIntake;
import frc.robot.commands.SnapDrivebaseToAngle;
import frc.robot.generated.WoodBotDriveTrain;
import frc.robot.subsystems.AlgaeArm.AlgaeArm;
import frc.robot.subsystems.AlgaeRoller.AlgaeRoller;
import frc.robot.subsystems.AlgaeShooter.AlgaeShooter;
import frc.robot.subsystems.AlgaeTilt.AlgaeTilt;
import frc.robot.subsystems.ClimberWinch.ClimberWinch;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.CoralShooter.CoralShooter;
import frc.robot.subsystems.Elevator.Elevator;
import frc.robot.subsystems.Funnel.Funnel;
import frc.robot.subsystems.Servo.Servo;
import frc.robot.subsystems.Vision.Vision;
import frc.robot.utils.CommandLogger;
import java.util.Map;
import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;
import org.opencv.calib3d.StereoBM;

// ↓ add docs ↓ there ↓ //
public class CommandFactory {
    private final CoralShooter coralShooter;
    private final Elevator elevator;
    private final Vision vision;
    private final ClimberWinch climberWinch;
    private final AlgaeShooter algaeShooter;
    private final AlgaeArm algaeArm;
    private final AlgaeRoller algaeRoller;
    private final CommandSwerveDrivetrain drivetrain;
    private final CommandXboxController driverCont;
    private final AlgaeTilt algaeTilt;
    private final Servo servo;
    private final Funnel funnel;

    double angleIncrement;
    double setpointIncrement;

    private final Timer climbTimer;

    // ↓ constructor ↓ //
    public CommandFactory(
            CoralShooter coralShooter,
            Elevator elevator,
            Vision vision,
            ClimberWinch climberWinch,
            AlgaeShooter algaeShooter,
            AlgaeArm algaeArm,
            CommandSwerveDrivetrain driveTrain,
            CommandXboxController driverCont,
            AlgaeTilt algaeTilt,
            AlgaeRoller algaeRoller,
            Servo servo,
            Funnel funnel) {
        this.coralShooter = coralShooter;
        this.elevator = elevator;
        this.vision = vision;
        this.climberWinch = climberWinch;
        this.algaeShooter = algaeShooter;
        this.algaeArm = algaeArm;
        this.drivetrain = driveTrain;
        this.driverCont = driverCont;
        this.algaeTilt = algaeTilt;
        this.algaeRoller = algaeRoller;
        this.servo = servo;
        this.funnel = funnel;
        this.climbTimer = new Timer();
        setpointIncrement = 0.0;
        angleIncrement = 0.0;
    }

    public Command alignRumble(CommandXboxController controller) {
        return CommandLogger.logCommand(
                Commands.runEnd(
                        () -> controller.getHID().setRumble(GenericHID.RumbleType.kBothRumble, 0.15),
                        () -> controller.getHID().setRumble(GenericHID.RumbleType.kBothRumble, 0.0)),
                "rumbling");
    }

    public Command intakeRumble(CommandXboxController controller) {
        return CommandLogger.logCommand(
                Commands.runEnd(
                        () -> controller.getHID().setRumble(GenericHID.RumbleType.kBothRumble, 0.75),
                        () -> controller.getHID().setRumble(GenericHID.RumbleType.kBothRumble, 0.0)),
                "rumbling");
    }

    public Command smartIntake() {
        return CommandLogger
                .logCommand(
                        SmartIntake.newCommand(coralShooter, funnel)
                                .andThen(this.intakeRumble(driverCont)
                                .withTimeout(0.2)),
                        "Smart Intake with Rumble");
    }



    /*
     * height is in motor rotations!
     */
    public Command setElevatorHeight(double height) {
        return CommandLogger.logCommand(
                elevator.isAtHeight(height).deadlineFor(elevator.setElevatorHeight(height)),
                "SetElevatorHeight");
    }

    public double calculateHeight(double height) {

        // if (vision.getTYRaw(CompBotConstants.CORAL_LIMELIGHT_NAME) < 15.0) {
        // double adjustment = 0.5 *
        // (vision.getTYRaw(CompBotConstants.CORAL_LIMELIGHT_NAME) - 12.3);
        // newHeight += adjustment;
        // }
        double adjustment = 0.25 * (vision.getTYRaw(CompBotConstants.CORAL_LIMELIGHT_NAME) - 12.3);
        height += adjustment;

        Logger.recordOutput("ADJUSTMENT HEIGHT", adjustment);
        Logger.recordOutput("CALCULATED ELEVATOR HEIGHT", height);

        return height;
    }

    public Command runElevatorVision(double height, DoubleSupplier tySupplier) {
        return Commands.runEnd(
                () -> setElevatorHeight(height + calculateHeight(height)),
                () -> setElevatorHeight(height + calculateHeight(height)));
    }

    public Command scalingElevatorHeight(double height) {
        double newHeight = height;
        if (vision.getTYRaw(CompBotConstants.CORAL_LIMELIGHT_NAME) < 15.0) {
            double adjustment = 0.5 * (vision.getTYRaw(CompBotConstants.CORAL_LIMELIGHT_NAME) - 12.3);
            newHeight += adjustment;
        }

        Logger.recordOutput("SCALING ELEVATOR HEIGHT", newHeight);

        return CommandLogger.logCommand(
                elevator.isAtHeight(newHeight).deadlineFor(elevator.setElevatorHeight(newHeight)),
                "ScalingSetElevatorHeight");
    }

    public Command setElevatorLevelFour() {
        return setElevatorHeight(SetPointConstants.ElevatorHeights.TELE_LEVEL_FOUR);
    }

    public Command setElevatorLevelThree() {
        return setElevatorHeight(SetPointConstants.ElevatorHeights.TELE_LEVEL_THREE);
    }

    public Command setElevatorLevelTwo() {
        return setElevatorHeight(SetPointConstants.ElevatorHeights.TELE_LEVEL_TWO);
    }

    public Command setElevatorLevelOne() {
        return setElevatorHeight(SetPointConstants.ElevatorHeights.TELE_LEVEL_ONE);
    }

    public Command setElevatorToZero() {
        return setElevatorHeight(0.0);
    }

    public Command setElevatorHeightZeroAndZero() {
        return new SequentialCommandGroup(setElevatorToZero(), elevator.zeroElevatorCmd());
    }

    public Command setAlgaeArmAngle(double angle) {
        return algaeArm.setAlgaeArmAngleCmd(angle);
    }

    public Command setAlgaeTiltPosition(double position) {
        return algaeTilt.setPositionCmd(position);
    }

    /**
     *
     * @param goalTY
     * @param goalTX
     * @param pipeline 0 is right, 1 is left
     * @return
     */
    public Command teleAlignWithLimelight(
            double goalTY,
            double goalTX,
            int pipeline,
            CommandXboxController driverCont) {
        return CommandLogger
                .logCommand(
                        new AlignWithLimelight(vision, drivetrain, goalTY, goalTX, pipeline, driverCont),
                        "AlignWithLimelightBase")
                .andThen(this.alignRumble(driverCont)
                        .onlyIf(() -> vision.getTV(Constants.CompBotConstants.CORAL_LIMELIGHT_NAME) == 1)
                        .withTimeout(0.1));
    }

    public Command autoAlignWithLimelight(double goalTY, double goalTX, int pipeline) {
        return CommandLogger
                .logCommand(
                        new AlignWithLimelight(vision, drivetrain, goalTY, goalTX, pipeline),
                        "AlignWithLimelightAuto");
    }

    public Command pidAlign(boolean isRight) {
        return CommandLogger
                .logCommand(PIDToReefPoints.pidToReef(drivetrain, () -> drivetrain.getPose(), isRight),
                        "PID Align")
                .andThen(this.alignRumble(driverCont)
                        .withTimeout(0.2));
    }

    /**
     * This method is to reliably align the drivebase with the limelight
     * It repeatedly attempts to align the robot with the targetted position and
     * then ends finally when the robot is at the appropriate heading and tx/ty
     * positions
     * uses the coral limelight
     *
     * @param isLeft is it on the left reef (true) or right reef (false)
     * @return
     */
    public Command alignWithLimelightAutomated(boolean isLeft) {
        double goalTY = Constants.WoodbotConstants.WBGOALSCORETY;
        double goalTX = Constants.WoodbotConstants.WBGOALSCORETX;
        int pipeline = isLeft ? 1 : 0;

        return Commands
                .waitUntil(
                        () -> {
                            boolean onTX = drivetrain.strafeController.atSetpoint();
                            boolean onTY = drivetrain.forwardController.atSetpoint();
                            boolean onHeading = drivetrain.isAtRotationSetpoint();

                            String cmdTag = "AlignWithLimelightAutomated: ";
                            Logger.recordOutput(cmdTag + "onTX", onTX);
                            Logger.recordOutput(cmdTag + "onTY", onTY);
                            Logger.recordOutput(cmdTag + "onHeading", onHeading);
                            return (onTX &&
                                    onTY &&
                                    onHeading &&
                                    vision.isTargetInView(Constants.PracticeBotConstants.CORAL_LIMELIGHT_NAME));
                        })
                .deadlineFor(teleAlignWithLimelight(goalTY, goalTX, pipeline, driverCont).repeatedly());
    }

    /**
     * This aligns the robot to the target and raises the elevator to specified
     * level and then runs the coral shooter
     *
     * @param level
     * @param isLeft
     * @return
     */
    public Command scoringRoutine(int level, boolean isLeft) {
        return alignWithLimelightAutomated(isLeft)
                .andThen(
                        new SelectCommand<Integer>(
                                Map.ofEntries(
                                        Map.entry(1, setElevatorLevelOne()),
                                        Map.entry(2, setElevatorLevelTwo()),
                                        Map.entry(3, setElevatorLevelThree()),
                                        Map.entry(4, setElevatorLevelFour())),
                                () -> level)
                                .raceWith(drivetrain.xOutCmd()))
                .andThen(coralShooter.basicShootCmd().raceWith(drivetrain.xOutCmd()));
    }

    public Command scoreLevelOne() {
        return setElevatorLevelOne().andThen(coralShooter.basicShootCmd());
    }

    public Command scoringRoutineTeleop(int level, boolean isLeft) {
        return scoringRoutine(level, isLeft).andThen(setElevatorHeightZeroAndZero());
    }

    public Command hasCoral(Elevator elevator, CoralShooter coralShooter) {
        return Commands.either(
                new SequentialCommandGroup(
                        elevator.setElevatorHeight(ElevatorHeights.AUTO_LEVEL_FOUR),
                        coralShooter.basicShootCmd()),
                Commands.none(),
                () -> coralShooter.getIntakeSensor() || coralShooter.getOuttakeSensor());
    }

    public Command alignToReefWoodbotLeft(int pipeline) {
        return new SequentialCommandGroup(
                new SnapDrivebaseToAngle(vision, drivetrain, pipeline),
                new AlignWithLimelight(
                        vision,
                        drivetrain,
                        -12.64,
                        -11.16,
                        0,
                        new CommandXboxController(0)));
    }

    private boolean climberDeployed = false;

    public Command homeAlgaeTilt() {
        return Commands.either(
                algaeTilt.setPositionCmd(Constants.isCompBot() ? 0.07 : 0.07), // used to be 10, 4 works
                // for some reason 3/15
                algaeTilt.setPositionCmd(0.907),
                () -> !climberDeployed);
    }

    public Command groundPickupAlgaeTilt() {
        return algaeTilt.setPositionCmd(Constants.isCompBot() ? 0.3 : 35.0);
    }

    public Command driverIntakeAlgae() {
        return algaeRoller
                .setDutyCycleCmd(-0.1)
                .alongWith(algaeShooter.setDutyCycleCmd(-1.0))
                .alongWith(algaeTilt.setPositionCmd(Constants.isCompBot() ? 0.32 : 0.32));
    }

    public Command driverLollipopIntake() {
        return algaeRoller
                .setDutyCycleCmd(-0.1)
                .alongWith(algaeShooter.setDutyCycleCmd(-1.0))
                .alongWith(algaeTilt.setPositionCmd(0.205));
    }

    public Command driverProcessAlgae() {
        return algaeTilt
                .setPositionCmd(Constants.isCompBot() ? 0.253 : 0.253)
                .alongWith(algaeShooter.setDutyCycleCmd(0.6))
                .alongWith(algaeRoller.setDutyCycleCmd(0.8));
    }

    public Command operatorIntakeAlgae() {
        return algaeRoller.setDutyCycleCmd(-0.5).alongWith(algaeShooter.setDutyCycleCmd(-1.0));
    }

    public Command operatorOutakeAlgae() {
        return algaeShooter.setDutyCycleCmd(0.9);
    }

    // public Command processOrShoot() {
    // if (algaeTilt.getPositionRelative() >= 19.0 ||
    // algaeTilt.getPositionAbsolute() >= 0.2) {
    // return operatorOutakeAlgae();
    // } else {
    // return shootAlgae();
    // }
    // }

    // public Command limelightShootAlgae() {
    // InterpolatingDoubleTreeMap distanceVelocity = new
    // InterpolatingDoubleTreeMap();
    // distanceVelocity.put(2.91, 5000.0);
    // distanceVelocity.put(14.13, 5500.0);
    // distanceVelocity.put(9.14, 5250.0);

    // InterpolatingDoubleTreeMap distanceAngle = new InterpolatingDoubleTreeMap();
    // distanceAngle.put(2.91, 0.035);
    // distanceAngle.put(14.13, 0.055);
    // distanceAngle.put(9.14, 0.05);

    // double setPoint =
    // distanceVelocity.get(vision.getTYRaw(CompBotConstants.ALGAE_LIMELIGHT_NAME));
    // double tolerance = 50;

    // Logger.recordOutput("Algae RPM", setPoint);
    // Logger.recordOutput("Algae Angle",
    // distanceAngle.get(vision.getTYRaw(CompBotConstants.ALGAE_LIMELIGHT_NAME)));

    // return Commands
    // .waitUntil(
    // () -> {
    // return (Math.abs(algaeShooter.getVelocity() - setPoint) < tolerance);
    // }
    // )
    // .andThen(algaeRoller.setDutyCycleCmd(1.0))
    // .alongWith(algaeShooter.setVelocityCmd(setPoint))
    // .alongWith(algaeTilt.setPositionCmd(distanceAngle.get(vision.getTYRaw(CompBotConstants.ALGAE_LIMELIGHT_NAME))));
    // //old number 0.028, 0.057, 0.07
    // }


    public Command shootAlgae() {

        double setPoint = 4700.0; // 6000,
        double angle = 0.035;

        Logger.recordOutput("shootalgaesetpoint", setPoint);
        Logger.recordOutput("shootalgaeangle", angle);

        double tolerance = 50.0;
        return Commands
                .waitUntil(
                        () -> {
                            return (Math.abs(algaeShooter.getVelocity() - setPoint) < tolerance);
                        })
                .andThen(algaeRoller.setDutyCycleCmd(1.0))
                .alongWith(algaeShooter.setVelocityCmd(setPoint))
                .alongWith(algaeTilt.setPositionCmd(angle)); // old number 0.028, 0.057,
                                                             // 0.07
    }

    public void upAngle() {
        angleIncrement += 0.005;
    }

    public void downAngle() {
        angleIncrement -= 0.005;
    }

    public void upSetPoint() {
        setpointIncrement += 25.0;
    }

    public void downSetPoint() {
        setpointIncrement -= 25.0;
    }

    public Command shootNoAngle() {
        return Commands
                .waitUntil(() -> algaeShooter.getVelocity() > 5750)
                .andThen(algaeRoller.setDutyCycleCmd(1.0))
                .alongWith(algaeShooter.setVelocityCmd(6250))
                .alongWith(algaeTilt.setPositionCmd(Constants.isCompBot() ? 0.03 : 0.03));
    }

    public Command processAndScore() {
        return algaeTilt
                .setPositionCmd(Constants.isCompBot() ? 0.253 : 0.253)
                .alongWith(this.shootAlgae());
    }

    public Command spinUpAlgaeShooter() {
        return algaeShooter.setVelocityCmd(6250.0);
    }

    /**
     * This command assumes the elevator is already above the algae
     *
     * @return
     */
    public Command intakeAlgaeFromReef() {
        return algaeArm
                .setAlgaeArmAngleCmd(110.0)
                .alongWith(coralShooter.pullAlgae())
                .alongWith(algaeShooter.setDutyCycleCmd(-0.8))
                .alongWith(algaeTilt.setPositionCmd(0.0))
                .alongWith(
                        Commands
                                .waitUntil(() -> coralShooter.getVelocity() < -6000.0)
                                .andThen(
                                        elevator.setElevatorHeight(
                                                SetPointConstants.ElevatorHeights.TELE_LEVEL_THREE - 3.0)));
    }

    public Command removeAlgaeL2() {
        return removeAlgae(2);
    }

    public Command removeAlgaeL3() {
        return removeAlgae(3);
    }

    public Command retractAlgaeArm() {
        return this.setAlgaeArmAngle(10.0);
    }

    public Command extendAlgaeArm() {
        return this.setAlgaeArmAngle(110.0);
    }

    private Command removeAlgae(int level) { // NOT BEING USED
        double height;
        if (level == 2) {
            height = ElevatorHeights.TELE_LEVEL_THREE - 6.0; // - 3.0 rotations from L4
        } else {
            height = ElevatorHeights.TELE_LEVEL_FOUR - 6.5; // - 3.0 rotations from L3
        }

        algaeArm.setAlgaeArmAngleCmd(60.0);

        if (coralShooter.getVelocity() < -6000.0) {
            return elevator.setElevatorHeight(height);
        } else {
            return coralShooter.pullAlgae();
        }
    }

    public Command climberSetupAlgaeTilt() {
        return algaeTilt.setPositionCmd(0.25);
    }

    public Command deployClimb() {
        return Commands
                .waitUntil(() -> (climbTimer.get() > 2.0))
                .deadlineFor(
                        servo
                                .runWithTimeout(2.0, 0)
                                .alongWith(new InstantCommand(() -> climbTimer.reset()))
                                .alongWith(new InstantCommand(() -> climbTimer.start()))
                                .alongWith(algaeTilt.setPositionCmd(0.256))
                                .andThen(
                                        new InstantCommand(() -> System.out.println("TIMEOUT IS DONE HERERERE"))) // 20
                                                                                                                  // for
                                                                                                                  // practice
                                .andThen(new InstantCommand(() -> this.climberDeployed = true)));
    }

    double climberWinchSetPoint = -33.00; // -34.28 new number 4:53pm 4-1-2025

    public Command initiateClimb() {
        return Commands
                .waitUntil(() -> climberWinch.getPosition() < climberWinchSetPoint + 1.0)
                .deadlineFor(climberWinch.setDutyCycleCmd(-0.8))
                .alongWith(algaeTilt.setPositionCmd(0.907)); // -5 for comp bot
    }

    public Command depolyAndInitiateClimb() {
        return deployClimb().andThen(initiateClimb());
    }

    public Command climb() {
        return climberWinch.setDutyCycleCmd(-0.8);
    }

    public Command operatorClimb() {
        return climb().alongWith(algaeTilt.setPositionCmd(0.907));
    }

    public Command climbAutomated() {
        return Commands
                .waitUntil(() -> climberWinch.getPosition() < -140.0)
                .deadlineFor(climb())
                .alongWith(algaeTilt.setPositionCmd(0.907));
    }

    public void resetClimberDeployed() {
        climberDeployed = false;
    }

    // public Command calculateWheelRadius() {
    //     drivetrain.zero();
    //     double startHeading = drivetrain.getAngle();
    //     double currentDrivePosition = drivetrain.getModule(1).getDriveMotor().getPosition().getValueAsDouble();
    //     drivetrain.rotateDrivetrain();
    //     double currentHeading = 0.0;
    //     waitUntil()

    // }
}
