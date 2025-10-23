// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Funnel;

import org.littletonrobotics.junction.AutoLog;

public interface FunnelIO {
    /** Creates a new FunnelIO. */
    @AutoLog
    public static class FunnelIOInputs {
        public double funnelCurrent = 0.0;
        public double funnelVoltage = 0.0;
        public double funnelVelocity = 0.0;
    }

    public default void updateInputs(FunnelIOInputs inputs) {}

    public void setDutyCycle(double dutyCycle);

    public void stop();
}
