// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.led.*;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CompBotConstants;

public class Lights extends SubsystemBase {
    private final CANdle candle = new CANdle(CompBotConstants.CANDLE_ID);

    /** Creates a new LEDs. */
    public Lights() {
        candle.configLEDType(LEDStripType.GRB);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        candle.setLEDs(200, 200, 200);
    }
}
