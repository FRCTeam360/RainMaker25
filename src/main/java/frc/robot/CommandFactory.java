package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.AlignWithLimelight;
import frc.robot.commands.SnapDrivebaseToAngle;
import frc.robot.subsystems.CommandSwerveDrivetrain;
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
    private final CommandSwerveDrivetrain drivetrain;
    
    // ↓ constructor ↓ //
    public CommandFactory(
        Catapult catapult,
        CoralIntake coralIntake,
        CoralShooter coralShooter,
        Elevator elevator,
        Vision vision,
        CommandSwerveDrivetrain driveTrain
    ) 
    
    {
        this.catapult = catapult;
        this.coralIntake = coralIntake;
        this.coralShooter = coralShooter;
        this.elevator = elevator;
        this.vision = vision;
        this.drivetrain = driveTrain;
    }

    /*
     * height is in motor rotations!
     */
    public Command setElevatorHeight(double height) {
        return elevator.setElevatorHeight(height);
    }

    public Command allignToReefWoodbotLeft(){
        new SequentialCommandGroup(
            new SnapDrivebaseToAngle(drivetrain, 0, vision),
            new AlignWithLimelight(vision, drivetrain, 3.0, 0.0, 0.25, Constants.OldCompBotConstants.maxAngularRate)
        );
        return allignToReefWoodbotLeft();
    }
}
