// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Servo;

import edu.wpi.first.wpilibj.Servo;
import frc.robot.Constants;

public class ServoIOCB implements ServoIO {
    private final Servo servo = new Servo(Constants.CompBotConstants.SERVO_PORT);

    /** Creates a new ServoIOCB. */
    public ServoIOCB() {}

    public void setServoSpeed(double speed) {
        servo.setPosition(speed);
    }

    public void updateInputs(ServoIOInputs inputs) {
        inputs.setOutput = servo.getPosition();
    }
}
