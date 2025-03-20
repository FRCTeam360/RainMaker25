// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.Constants.*;
import frc.robot.subsystems.AlgaeRoller.AlgaeRoller;
import frc.robot.subsystems.AlgaeShooter.AlgaeShooter;
import frc.robot.subsystems.AlgaeTilt.AlgaeTilt;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Vision.Vision;
import java.util.Map;
import java.util.Optional;
import org.littletonrobotics.junction.Logger;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class BargeAlign extends Command {
    private final CommandSwerveDrivetrain driveTrain;
    private final CommandXboxController cont;
    private final AlgaeShooter algaeShooter;
    private final AlgaeTilt algaeTilt;
    private final AlgaeRoller algaeRoller;
    private final Vision vision;
    private boolean endEarly = false;

    private double angle = 0.0;
    private double goalTY = 0.5;
    private int id;

    private final String CMD_NAME = "Barge Align: ";

    private static final Map<Integer, Double> tagIDToAngle = Map.ofEntries( //blue pov
        Map.entry(15, -90.0),
        Map.entry(14, -90.0),
        Map.entry(4, 90.0),
        Map.entry(5, 90.0)
    );

    /** Creates a new BargeAlign. */
    public BargeAlign(
        CommandSwerveDrivetrain driveTrain,
        Vision vision,
        AlgaeShooter algaeShooter,
        AlgaeTilt algaeTilt,
        AlgaeRoller algaeRoller,
        CommandXboxController cont
    ) {
        this.driveTrain = driveTrain;
        this.vision = vision;
        this.algaeShooter = algaeShooter;
        this.cont = cont;
        this.algaeTilt = algaeTilt;
        this.algaeRoller = algaeRoller;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(driveTrain, algaeShooter, algaeTilt, algaeRoller);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        System.out.println("Running");
        id = vision.getAprilTagID(Constants.isCompBot() ? CompBotConstants.ALGAE_LIMELIGHT_NAME
                : PracticeBotConstants.ALGAE_LIMELIGHT_NAME);
        
        endEarly = true;
        if (vision.getTV(Constants.isCompBot() ? CompBotConstants.ALGAE_LIMELIGHT_NAME
        : PracticeBotConstants.ALGAE_LIMELIGHT_NAME) == 1 && tagIDToAngle.containsKey(id)) {
                endEarly = false;
                angle = tagIDToAngle.get(id);
        }
        if (endEarly) return;

        Logger.recordOutput(CMD_NAME + "angle", angle);
        state = AlgaeShooterStates.SET_POSE_AND_ROT;
    }
    private enum AlgaeShooterStates{
        SET_POSE_AND_ROT,
        SHOOT
    }

    private AlgaeShooterStates state = AlgaeShooterStates.SET_POSE_AND_ROT;


    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if(driveTrain.forwardController.atSetpoint() && driveTrain.headingController.atSetpoint() && algaeShooter.getVelocity() > 5750){
            state = AlgaeShooterStates.SHOOT;
        }else{
            state = AlgaeShooterStates.SET_POSE_AND_ROT;
        }
        Logger.recordOutput(CMD_NAME + "State " , state);

        switch(state){
            case SET_POSE_AND_ROT:
                setPose();
                break;
            case SHOOT:
                setPose();
                algaeRoller.setDutyCycle(1.0);
                break;
        }
       // System.out.println("Executing");
        // return Commands.waitUntil(() -> algaeShooter.getVelocity() > 5750)
        // .andThen(algaeRoller.setDutyCycleCmd(1.0))
        // .alongWith(algaeShooter.setVelocityCmd(6250))
        // .alongWith(algaeTilt.setPositionCmd(Constants.isCompBot() ? 0.03 : 3.0));

    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        algaeRoller.setDutyCycle(0.0);
        algaeShooter.setDutyCycle(0.0);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        // //shooter is at vel angle is good ty is good
        // boolean onTY = driveTrain.forwardController.atSetpoint();
        // boolean atAngle = driveTrain.headingController.atSetpoint();
        // boolean atVelocity = algaeShooter.getVelocity() > 5750;

        // Logger.recordOutput(CMD_NAME + "onTY", onTY);
        // Logger.recordOutput(CMD_NAME + "atAngle", atAngle);
        // Logger.recordOutput(CMD_NAME + "atVelocity", atVelocity);

        // return onTY && atAngle && atVelocity;
        return endEarly;
    }
    private void setPose(){
        algaeShooter.setVelocity(6250.0);
        algaeTilt.setPosition(Constants.isCompBot() ? 0.03 : 3.0);

        double velX = driveTrain.forwardController.calculate(
            vision.getTYRaw(PracticeBotConstants.ALGAE_LIMELIGHT_NAME),
            goalTY,
            driveTrain.getState().Timestamp
        );

        velX = velX * -Math.signum(angle);

        Logger.recordOutput(CMD_NAME + "velX", velX);

        double velY = Math.pow(MathUtil.applyDeadband(-cont.getLeftX(), 0.1), 3)  * driveTrain.getMaxSpeed();

        Optional<Alliance> alliance = DriverStation.getAlliance();
        if (alliance.isPresent() && alliance.get() == Alliance.Red) {
            velY = -velY;
        }
        Logger.recordOutput(CMD_NAME + "vely", velY);


        driveTrain.driveFieldCentricFacingAngleBluePerspective(velX, velY, angle);
    }
}
