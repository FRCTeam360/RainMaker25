package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimberWinch.ClimberWinch;
import frc.robot.subsystems.CoralIntake.CoralIntake;
// import frc.robot.subsystems.CoralShooter.CoralShooter;
import frc.robot.subsystems.CoralIntake.CoralIntake;
import frc.robot.subsystems.CoralShooter.CoralShooter;
import frc.robot.subsystems.Elevator.Elevator;
import frc.robot.subsystems.Vision.Vision;

// ↓ add docs ↓ there ↓ //
public class CommandFactory {
    private final CoralIntake coralIntake;
    private final CoralShooter coralShooter;
    private final Elevator elevator;
    private final Vision vision; 
    private final ClimberWinch climber;

    // ↓ constructor ↓ //
public CommandFactory(
    CoralIntake coralIntake,
    CoralShooter coralShooter,
    Elevator elevator,
    Vision vision,
    ClimberWinch climber
) {
    this.coralIntake = coralIntake;
    this.coralShooter = coralShooter;
    this.elevator = elevator; 
    this.vision = vision;
    this.climber = climber;
}

public Command setElevatorHeight(double height) {
    return elevator.setElevatorHeight(height);
}

}
