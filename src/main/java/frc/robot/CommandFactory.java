package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AlgaeArm.AlgaeArm;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.AlignWithLimelight;
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
    private final AlgaeArm algaeArm;
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
        AlgaeArm algaeArm,
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
        this.algaeArm = algaeArm;
        this.drivetrain = driveTrain;
        this.driverCont = driverCont;
    }

    /*
     * height is in motor rotations!
     */
    public Command setElevatorHeight(double height) {
        return elevator.setElevatorHeight(height);
    }

public Command setAlgaeArmAngle(double angle) {
    return algaeArm.setAlgaeArmAngle(angle);
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

    public Command alignToReefWoodbotLeft(){
        return new SequentialCommandGroup(
            new SnapDrivebaseToAngle(drivetrain),
            new AlignWithLimelight(vision, drivetrain, -12.64, -11.16, 0)
        );
    }
}
