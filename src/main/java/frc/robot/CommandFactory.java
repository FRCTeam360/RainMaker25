package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;

import java.util.Map;

import org.littletonrobotics.junction.Logger;
import org.opencv.calib3d.StereoBM;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AlgaeArm.AlgaeArm;
import frc.robot.subsystems.AlgaeRoller.AlgaeRoller;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SelectCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.AlignWithLimelight;
import frc.robot.commands.SetCoralIntake;
import frc.robot.commands.SnapDrivebaseToAngle;
import frc.robot.generated.WoodBotDriveTrain;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimberWinch.ClimberWinch;
import frc.robot.subsystems.AlgaeShooter.AlgaeShooter;
import frc.robot.subsystems.AlgaeTilt.AlgaeTilt;
import frc.robot.subsystems.ClimberWheel.ClimberWheel;
import frc.robot.subsystems.CoralIntake.CoralIntake;
// import frc.robot.subsystems.CoralShooter.CoralShooter;
import frc.robot.subsystems.CoralIntake.CoralIntake;
import frc.robot.subsystems.CoralShooter.CoralShooter;
import frc.robot.subsystems.Elevator.Elevator;
import frc.robot.subsystems.Vision.Vision;
import frc.robot.utils.CommandLogger;

// ↓ add docs ↓ there ↓ //
public class CommandFactory {
    private final CoralIntake coralIntake;
    private final CoralShooter coralShooter;
    private final Elevator elevator;
    private final Vision vision;
    private final ClimberWinch climberWinch;
    private final ClimberWheel climberWheel;
    private final AlgaeShooter algaeShooter;
    private final AlgaeArm algaeArm;
    private final CommandSwerveDrivetrain drivetrain;
    private final XboxController driverCont;
    private final AlgaeTilt algaeTilt;
    private final AlgaeRoller algaeRoller;

    // ↓ constructor ↓ //
    public CommandFactory(
            CoralIntake coralIntake,
            CoralShooter coralShooter,
            Elevator elevator,
            Vision vision,
            ClimberWinch climberWinch,
            ClimberWheel climberWheel,
            AlgaeShooter algaeShooter,
            AlgaeArm algaeArm,
            CommandSwerveDrivetrain driveTrain,
            XboxController driverCont,
            AlgaeTilt algaeTilt,
            AlgaeRoller algaeRoller) {
        this.coralIntake = coralIntake;
        this.coralShooter = coralShooter;
        this.elevator = elevator;
        this.vision = vision;
        this.climberWinch = climberWinch;
        this.climberWheel = climberWheel;
        this.algaeShooter = algaeShooter;
        this.algaeArm = algaeArm;
        this.drivetrain = driveTrain;
        this.driverCont = driverCont;
        this.algaeTilt = algaeTilt;
        this.algaeRoller = algaeRoller;
    }

    /*
     * height is in motor rotations!
     */
    public Command setElevatorHeight(double height) {
        return CommandLogger.logCommand(elevator.isAtHeight(height).deadlineFor(elevator.setElevatorHeight(height)),
                "SetElevatorHeight");
    }

    public Command setElevatorLevelFour() {
        return setElevatorHeight(29.5);
    }

    public Command setElevatorLevelThree() {
        return setElevatorHeight(16.0);
    }

    public Command setElevatorLevelTwo() {
        return setElevatorHeight(7.0);
    }

    public Command setElevatorToZero() {
        return setElevatorHeight(0.0);
    }

    public Command setElevatorLevelOne() {
        return setElevatorHeight(2.0);
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
    public Command alignWithLimelight(double goalTY, double goalTX, int pipeline) {
        return // vision.waitUntilTargetTxTy(goalTX,
               // goalTY).alongWith(drivetrain.waitUntilDrivetrainAtHeadingSetpoint())
        CommandLogger.logCommand(new AlignWithLimelight(vision, drivetrain, goalTY, goalTX,
                pipeline), "AlignWithLimelightBase"); // no more timeout
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

        return Commands.waitUntil(() -> {
            boolean onTX = drivetrain.strafeController.atSetpoint();
            boolean onTY = drivetrain.forwardController.atSetpoint();
            boolean onHeading = drivetrain.isAtRotationSetpoint();

            String cmdTag = "AlignWithLimelightAutomated: ";
            Logger.recordOutput(cmdTag + "onTX", onTX);
            Logger.recordOutput(cmdTag + "onTY", onTY);
            Logger.recordOutput(cmdTag + "onHeading", onHeading);
            return onTX && onTY && onHeading
                    && vision.isTargetInView(Constants.PracticeBotConstants.CORAL_LIMELIGHT_NAME);
        }).deadlineFor(alignWithLimelight(goalTY, goalTX, pipeline).repeatedly());
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
                .andThen(new SelectCommand<Integer>(Map.ofEntries(
                        Map.entry(1, setElevatorLevelOne()),
                        Map.entry(2, setElevatorLevelTwo()),
                        Map.entry(3, setElevatorLevelThree()),
                        Map.entry(4, setElevatorLevelFour())),
                        () -> level).raceWith(drivetrain.xOutCmd()))
                .andThen(coralShooter.basicShootCmd().raceWith(drivetrain.xOutCmd()));
    }

    public Command scoreLevelOne() {
        return setElevatorLevelOne().andThen(coralShooter.basicShootCmd());
    }

    public Command scoringRoutineTeleop(int level, boolean isLeft) {
        return scoringRoutine(level, isLeft).andThen(setElevatorHeightZeroAndZero());
    }

    public Command alignToReefWoodbotLeft(int pipeline) {
        return new SequentialCommandGroup(
                new SnapDrivebaseToAngle(vision, drivetrain, pipeline),
                new AlignWithLimelight(vision, drivetrain, -12.64, -11.16, 0));
    }

    public Command intakeAlgaeFromGround() {
        return algaeRoller.setDutyCycleCmd(-0.3)
                .alongWith(algaeShooter.setDutyCycleCmd(1.0));
    }

    public Command outtakeAlgaeFromGround() {
        return algaeRoller.setDutyCycleCmd(0.3)
                .alongWith(algaeShooter.setDutyCycleCmd(-0.8)).alongWith(algaeTilt.setPositionCmd(30.0));
    }
}