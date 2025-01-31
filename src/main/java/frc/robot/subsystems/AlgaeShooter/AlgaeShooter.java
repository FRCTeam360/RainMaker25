package frc.robot.subsystems.AlgaeShooter;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AlgaeShooter extends SubsystemBase{
    private final AlgaeShooterIO io;
    private final AlgaeShooterIOInputsAutoLogged inputs = new AlgaeShooterIOInputsAutoLogged();

    //Creates AlgaeShooter
    public AlgaeShooter(AlgaeShooterIO io) {
        this.io = io;
    }

    @Override
    public void periodic(){
        io.updateInputs(inputs);
        Logger.processInputs("Algae Shooter", inputs);
    }
}