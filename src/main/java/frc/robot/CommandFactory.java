package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimberWinch.ClimberWinch;
import frc.robot.subsystems.AlgaeShooter.AlgaeShooter;
import frc.robot.subsystems.ClimberWheel.ClimberWheel;
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
    private final ClimberWinch climberWinch;
    private final ClimberWheel climberWheel;
    private final AlgaeShooter algaeShooter;

    // ↓ constructor ↓ //
public CommandFactory(
    CoralIntake coralIntake,
    CoralShooter coralShooter,
    Elevator elevator,
    Vision vision,
    ClimberWinch climberWinch,
    ClimberWheel climberWheel,
    AlgaeShooter algaeShooter
) {
    this.coralIntake = coralIntake;
    this.coralShooter = coralShooter;
    this.elevator = elevator; 
    this.vision = vision;
    this.climberWinch = climberWinch;
    this.climberWheel = climberWheel;
    this.algaeShooter = algaeShooter;
}

public Command setElevatorHeight(double height) {
    return elevator.setElevatorHeight(height);
}

}
