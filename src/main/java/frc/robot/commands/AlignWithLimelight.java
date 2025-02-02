// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Vision.Vision;
import java.util.function.DoubleSupplier;

import javax.annotation.processing.SupportedOptions;

import org.littletonrobotics.junction.Logger;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AlignWithLimelight extends Command {
    private Vision vision;
    private CommandSwerveDrivetrain driveTrain;
    private double goalTY;
    private double goalTX;
    private double maxSpeed;
    private double maxAngularRate;
    private double angleToFace = 0.0;
    private XboxController driverCont;
    private double lastTXValue;
    private double lastTYValue;
    private Rotation2d angleToFaceRotation2d; 
    private Translation2d PIDSpeed;
    private double velocityX;
    private double velocityY;
    /** Creates a new AlignWithLimelight. */
    public AlignWithLimelight(
        Vision vision,
        CommandSwerveDrivetrain driveTrain,
        double goalTY,
        double goalTX,
        double maxSpeed
    ) {
        this.vision = vision;
        this.driveTrain = driveTrain;
        this.goalTY = goalTY;
        this.maxSpeed = maxSpeed;
        this.goalTX = goalTX;

        addRequirements(driveTrain);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {

        if (vision.getAprilTagID() == 21 || vision.getAprilTagID() == 7) {
            angleToFace = 180.0;
        } else if (vision.getAprilTagID() == 22 || vision.getAprilTagID() == 6)  {
            angleToFace = 120.0;
        } else if (vision.getAprilTagID() == 17 || vision.getAprilTagID() == 11) {
            angleToFace = 60.0;
        } else if (vision.getAprilTagID() == 18 || vision.getAprilTagID() == 10) {
            angleToFace = 0.0;
        } else if (vision.getAprilTagID() == 19 || vision.getAprilTagID() == 9) {
            angleToFace = -60.0;
        } else if (vision.getAprilTagID() == 20 || vision.getAprilTagID() == 8) {
            angleToFace = -120.0; 
        }

        angleToFaceRotation2d = Rotation2d.fromDegrees(angleToFace);


        // if (driveTrain.getAngle() >= -30.0 || driveTrain.getAngle() <= 30.0) {
        //     angleToFace = 0.0;
        // } else if (driveTrain.getAngle() <= -30.0 || driveTrain.getAngle() >= -90.0) {
        //     angleToFace = -60.0;
        // } else if (driveTrain.getAngle() <= -90.0 || driveTrain.getAngle() >= -150.0) {
        //     angleToFace = -120.0;
        // } else if (driveTrain.getAngle() <= -150.0 || driveTrain.getAngle() >= 150.0) {
        //     angleToFace = 180.0;
        // } else if (driveTrain.getAngle() <= 150.0 || driveTrain.getAngle() >= 90.0) {
        //     angleToFace = 120.0;
        // } else if (driveTrain.getAngle() >= 30.0 || driveTrain.getAngle() <= 90.0) {
        //     angleToFace = 60.0;
        // }
        
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
     //   lastTXValue = vision.getTXRaw();
       // lastTYValue = vision.getTYRaw();
      // velocityX = 0 ;
        velocityX =  driveTrain.translationController.calculate(vision.getTYRaw(), goalTY) ;//forward & backward motion
        velocityY =  -  driveTrain.translationController.calculate(vision.getTXRaw(), goalTX); 


    //    Translation2d PIDSpeed = new Translation2d(
    //     driveTrain.translationController.calculate(vision.getTYRaw(), goalTY),        
    //     -driveTrain.translationController.calculate(vision.getTXRaw(), goalTX)
    //    );

    //    velocityX = PIDSpeed.rotateBy(angleToFaceRotation2d).getX();
    //    velocityY = PIDSpeed.rotateBy(angleToFaceRotation2d).getY();

    //    Logger.recordOutput("KxVelX Output", velocityX);
    //    Logger.recordOutput("KyVelY Output", velocityY);



     if(vision.getTV() == 1) {
            driveTrain.driveFieldCentricFacingAngle(
                velocityX, //forward & backward motion
                velocityY,
                    angleToFace, //side to side motion
                    maxSpeed
                );

        } else {
     //      driveTrain.driveFieldCentricFacingAngle(0.0, 0.0, angleToFace, maxSpeed);
        }
       

    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {}

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        // return false;
        return Math.abs( vision.getTXRaw() - goalTX) < 2.0;// &&  //Math.abs(vision.getTYRaw()  - goalTY) < 0.5;
    }//
}
