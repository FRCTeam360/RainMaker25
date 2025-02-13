package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;

import java.util.Map;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SelectCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.AlignWithLimelight;
import frc.robot.commands.SetCoralIntake;
import frc.robot.commands.SnapDrivebaseToAngle;
import frc.robot.generated.WoodBotDriveTrain;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AlgaeShooter.AlgaeShooter;
import frc.robot.subsystems.Catapult.Catapult;
import frc.robot.subsystems.CoralIntake.CoralIntake;
// import frc.robot.subsystems.CoralShooter.CoralShooter;
import frc.robot.subsystems.Catapult.Catapult;
import frc.robot.subsystems.CoralIntake.CoralIntake;
import frc.robot.subsystems.CoralShooter.CoralShooter;
import frc.robot.subsystems.Elevator.Elevator;
import frc.robot.subsystems.Vision.Vision;

// ↓ add docs ↓ there ↓ //
public class CommandFactory {
    private final Catapult catapult;
    private final CoralIntake coralIntake;
    private final CoralShooter coralShooter;
    private final Elevator elevator;
    private final Vision vision;
    private final AlgaeShooter algaeShooter;
    private final CommandSwerveDrivetrain drivetrain;
    private final XboxController driverCont;    
    // ↓ constructor ↓ //
    public CommandFactory(
        Catapult catapult,
        CoralIntake coralIntake,
        CoralShooter coralShooter,
        Elevator elevator,
        Vision vision,
        AlgaeShooter algaeShooter,
        CommandSwerveDrivetrain driveTrain,
        XboxController driverCont
    ) 
    
    {
        this.catapult = catapult;
        this.coralIntake = coralIntake;
        this.coralShooter = coralShooter;
        this.elevator = elevator;
        this.vision = vision;
        this.algaeShooter = algaeShooter;
        this.drivetrain = driveTrain;
        this.driverCont = driverCont;
    }

    /*
     * height is in motor rotations!
     */
    public Command setElevatorHeight(double height) {
        return elevator.isAtHeight(height).deadlineFor(elevator.setElevatorHeight(height));
    }

    public Command setElevatorLevelFour(){
        return setElevatorHeight(33.5);
    }
    
    public Command setElevatorLevelThree(){
        return setElevatorHeight(23.0);
    }
    
    public Command setElevatorLevelTwo(){
        return setElevatorHeight(10.5);
    }
    
    public Command setElevatorLevelOne(){
        return setElevatorHeight(0.0);
    }

    public Command setElevatorLevelOneAndZero(){
        return new SequentialCommandGroup(setElevatorLevelOne(), elevator.zeroElevatorCmd());
    }

    /**
     * 
     * @param goalTY
     * @param goalTX
     * @param pipeline 0 is right, 1 is left
     * @return
     */
    public Command alignWithLimelight(double goalTY, double goalTX, int pipeline) {
        return //vision.waitUntilTargetTxTy(goalTX, goalTY).alongWith(drivetrain.waitUntilDrivetrainAtHeadingSetpoint())
            (new AlignWithLimelight(vision, drivetrain, goalTY, goalTX,
                        pipeline)); // no more timeout
    }

    /**
     * This method is to reliably align the drivebase with the limelight
     * It repeatedly attempts to align the robot with the targetted position and then ends finally when the robot is at the appropriate heading and tx/ty positions
     * @param isLeft is it on the left reef (true) or right reef (false)
     * @return
     */
    public Command alignWithLimelightAutomated(boolean isLeft){
        double goalTY = Constants.WoodbotConstants.WBGOALSCORETY;
        double goalTX = Constants.WoodbotConstants.WBGOALSCORETX;
        int pipeline = isLeft ? 0 : 1;

        return Commands.waitUntil(()-> {
            boolean onTX = drivetrain.strafeController.atSetpoint();
            boolean onTY = drivetrain.forwardController.atSetpoint();
            boolean onHeading = drivetrain.isAtRotationSetpoint();

            String cmdTag = "AlignWithLimelightAutomated: ";
            Logger.recordOutput(cmdTag + "onTX", onTX);
            Logger.recordOutput(cmdTag + "onTY", onTY);
            Logger.recordOutput(cmdTag + "onHeading", onHeading);
            return onTX && onTY && onHeading && vision.isTargetInView();
        }).deadlineFor(alignWithLimelight(goalTY, goalTX, pipeline).repeatedly());
    }

    /**
     * This aligns the robot to the target and raises the elevator to specified level and then runs the coral shooter
     * @param level
     * @param isLeft
     * @return
     */
    public Command scoringRoutine(int level, boolean isLeft) {
        return alignWithLimelightAutomated(isLeft)
                .alongWith(new SelectCommand<Integer>(Map.ofEntries(
                        Map.entry(1, setElevatorLevelOne()),
                        Map.entry(2, setElevatorLevelTwo()),
                        Map.entry(3, setElevatorLevelThree()),
                        Map.entry(4, setElevatorLevelFour())),
                        () -> level)).andThen(coralShooter.shootCmd());
    }

    public Command scoringRoutineTeleop(int level, boolean isLeft){
        return scoringRoutine(level, isLeft).andThen(setElevatorLevelOneAndZero());
    }

    public Command alignToReefWoodbotLeft(){
        return new SequentialCommandGroup(
            new SnapDrivebaseToAngle(drivetrain),
            new AlignWithLimelight(vision, drivetrain, -12.64, -11.16, 0)
        );
    }
}
