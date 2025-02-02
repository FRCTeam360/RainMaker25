package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AlgaeArm.AlgaeArm;
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
    private final AlgaeArm algaeArm; 

    // ↓ constructor ↓ //
public CommandFactory(
    Catapult catapult,
    CoralIntake coralIntake,
    CoralShooter coralShooter,
    Elevator elevator,
    Vision vision,
    AlgaeArm algaeArm
) {
    this.catapult = catapult;
    this.coralIntake = coralIntake;
    this.coralShooter = coralShooter;
    this.elevator = elevator; 
    this.vision = vision;
    this.algaeArm = algaeArm;
}

public Command setElevatorHeight(double height) {
    return elevator.setElevatorHeight(height);
}

public Command setAlgaeArmAngle(double angle) {
    return algaeArm.setAlgaeArmAngle(angle);
}
}
