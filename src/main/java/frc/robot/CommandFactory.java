package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;

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
import frc.robot.commands.SetCoralIntake;
import frc.robot.commands.SmartIntake;
import frc.robot.commands.SnapDrivebaseToAngle;
import frc.robot.generated.WoodBotDriveTrain;
import frc.robot.subsystems.AlgaeArm.AlgaeArm;
import frc.robot.subsystems.AlgaeRoller.AlgaeRoller;
import frc.robot.subsystems.AlgaeShooter.AlgaeShooter;
import frc.robot.subsystems.AlgaeTilt.AlgaeTilt;
import frc.robot.subsystems.ClimberWheel.ClimberWheel;
import frc.robot.subsystems.ClimberWinch.ClimberWinch;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.CoralShooter.CoralShooter;
import frc.robot.subsystems.Elevator.Elevator;
import frc.robot.subsystems.Servo.Servo;
import frc.robot.subsystems.Vision.Vision;
import frc.robot.utils.CommandLogger;
import java.util.Map;
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
        Servo servo
    ) {
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
        this.climbTimer = new Timer();
    }

    public Command rumbleDriverController(CommandXboxController controller) {
        return CommandLogger.logCommand(
            Commands.runEnd(
                () -> controller.getHID().setRumble(GenericHID.RumbleType.kBothRumble, 0.15),
                () -> controller.getHID().setRumble(GenericHID.RumbleType.kBothRumble, 0.0)
            ),
            "rumbling"
        );
    }

    /*
     * height is in motor rotations!
     */
    public Command setElevatorHeight(double height) {
        return CommandLogger.logCommand(
            elevator.isAtHeight(height).deadlineFor(elevator.setElevatorHeight(height)),
            "SetElevatorHeight"
        );
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
    public Command alignWithLimelight(
        double goalTY,
        double goalTX,
        int pipeline,
        CommandXboxController driverCont
    ) {
        return CommandLogger
            .logCommand(
                new AlignWithLimelight(vision, drivetrain, goalTY, goalTX, pipeline, driverCont),
                "AlignWithLimelightBase"
            )
            .andThen(this.rumbleDriverController(driverCont).withTimeout(0.1));
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
                    return (
                        onTX &&
                        onTY &&
                        onHeading &&
                        vision.isTargetInView(Constants.PracticeBotConstants.CORAL_LIMELIGHT_NAME)
                    );
                }
            )
            .deadlineFor(alignWithLimelight(goalTY, goalTX, pipeline, driverCont).repeatedly());
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
                        Map.entry(4, setElevatorLevelFour())
                    ),
                    () -> level
                )
                .raceWith(drivetrain.xOutCmd())
            )
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
                coralShooter.basicShootCmd()
            ),
            Commands.none(),
            () -> coralShooter.getIntakeSensor() || coralShooter.getOuttakeSensor()
        );
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
                new CommandXboxController(0)
            )
        );
    }

    private boolean climberDeployed = false;

    public Command homeAlgaeTilt() {
        return Commands.either(
            algaeTilt.setPositionCmd(Constants.isCompBot() ? 0.07 : 7.2), // used to be 10, 4 works
            // for some reason 3/15
            algaeTilt.setPositionCmd(0.907),
            () -> !climberDeployed
        );
    }

    public Command groundPickupAlgaeTilt() {
        return algaeTilt.setPositionCmd(Constants.isCompBot() ? 0.3 : 35.0);
    }

    public Command driverIntakeAlgae() {
        return algaeRoller
            .setDutyCycleCmd(-0.1)
            .alongWith(algaeShooter.setDutyCycleCmd(-1.0))
            .alongWith(algaeTilt.setPositionCmd(Constants.isCompBot() ? 0.32 : 23.5));
    }

    public Command driverProcessAlgae() {
        return algaeTilt
            .setPositionCmd(Constants.isCompBot() ? 0.253 : 21)
            .alongWith(algaeShooter.setDutyCycleCmd(0.6))
            .alongWith(algaeRoller.setDutyCycleCmd(0.8));
    }

    public Command operatorIntakeAlgae() {
        return algaeRoller.setDutyCycleCmd(-0.1).alongWith(algaeShooter.setDutyCycleCmd(-1.0));
    }

    public Command operatorOutakeAlgae() {
        return algaeShooter.setDutyCycleCmd(0.9);
    }

    public Command processOrShoot() {
        if (algaeTilt.getPositionRelative() >= 19.0 || algaeTilt.getPositionAbsolute() >= 0.2) {
            return operatorOutakeAlgae();
        } else {
            return shootAlgae();
        }
    }

    public Command shootAlgae() {
        return Commands
            .waitUntil(() -> algaeShooter.getVelocity() > 5750)
            .andThen(algaeRoller.setDutyCycleCmd(1.0))
            .alongWith(algaeShooter.setVelocityCmd(6250))
            .alongWith(algaeTilt.setPositionCmd(Constants.isCompBot() ? 0.03 : 3.0));
    }

    public Command processAndScore() {
        return algaeTilt
            .setPositionCmd(Constants.isCompBot() ? 0.253 : 30)
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
                            SetPointConstants.ElevatorHeights.TELE_LEVEL_THREE - 3.0
                        )
                    )
            );
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
            .waitUntil(() -> (climbTimer.get() > 3.5))
            .deadlineFor(
                servo
                    .runWithTimeout(3.5, 0)
                    .alongWith(new InstantCommand(() -> climbTimer.reset()))
                    .alongWith(new InstantCommand(() -> climbTimer.start()))
                    .alongWith(algaeTilt.setPositionCmd(0.256))
                    .andThen(
                        new InstantCommand(() -> System.out.println("TIMEOUT IS DONE HERERERE"))
                    ) // 20
                    // for
                    // practice
                    .andThen(new InstantCommand(() -> this.climberDeployed = true))
            );
    }

    double climberWinchSetPoint = -44.33;

    public Command initiateClimb() {
        return Commands
            .waitUntil(() -> climberWinch.getPosition() < climberWinchSetPoint + 1.0)
            .deadlineFor(climberWinch.setDutyCycleCmd(-0.3))
            .alongWith(algaeTilt.setPositionCmd(0.907)); // -5 for comp bot
    }

    public Command depolyAndInitiateClimb() {
        return deployClimb().andThen(initiateClimb());
    }

    public Command climb() {
        return climberWinch.setDutyCycleCmd(-0.8);
    }

    public Command climbAutomated() {
        return Commands
            .waitUntil(() -> climberWinch.getPosition() < -160.0)
            .deadlineFor(climb())
            .alongWith(algaeTilt.setPositionCmd(0.907));
    }

    public void resetClimberDeployed() {
        climberDeployed = false;
    }
}
