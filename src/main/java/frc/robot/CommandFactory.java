package frc.robot;

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

    // ↓ constructor ↓ //
public CommandFactory(
    Catapult catapult,
    CoralIntake coralIntake,
    CoralShooter coralShooter,
    Elevator elevator,
    Vision vision
) {
    this.catapult = catapult;
    this.coralIntake = coralIntake;
    this.coralShooter = coralShooter;
    this.elevator = elevator; 
    this.vision = vision;
}


}
