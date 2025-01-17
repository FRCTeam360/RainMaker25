package frc.robot;

import frc.robot.subsystems.Catapult.Catapult;
import frc.robot.subsystems.CoralIntake.CoralIntake;
import frc.robot.subsystems.CoralOuttake.CoralOuttake;
import frc.robot.subsystems.Elevator.Elevator;
import frc.robot.subsystems.Vision.Vision;

// ↓ add docs ↓ there ↓ //
public class CommandFactory {
    private final Catapult catapult;
    private final CoralIntake coralIntake;
    private final CoralOuttake coralOuttake;
    private final Elevator elevator;
    private final Vision vision; 

    // ↓ constructor ↓ //
public CommandFactory(
    Catapult catapult,
    CoralIntake coralintake,
    CoralOuttake coralouttake,
    Elevator elevator,
    Vision vision
) {
    this.catapult = catapult;
    this.coralIntake = coralintake;
    this.coralOuttake = coralouttake;
    this.elevator = elevator; 
    this.vision = vision;
}


}
