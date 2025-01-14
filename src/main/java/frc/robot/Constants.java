// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.signals.ConnectedMotorValue;

import edu.wpi.first.hal.HALUtil;
import edu.wpi.first.util.function.BooleanConsumer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.generated.OldCompBot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */

public final class Constants {

    public static class OperatorConstants {
        public static final int kDriverControllerPort = 0;
    }

    public static class VisionConstants {
        public static final String WOODBOT_LIMELIGHT_NAME = "limelight";
        public static final double WOODBOT_YAW_FUDGE_FACTOR = 0;
        public static final double WOODBOT_PITCH_FUDGE_FACTOR = 0;
    }

    public static class OldCompBotConstants {
        public static final String OCB_LIMELIGHT_NAME = "limelight";
        public static final double OCB_YAW_FUDGE_FACTOR = 0;
        public static final double OCB_PITCH_FUDGE_FACTOR = 0;

        public static final double maxSpeed = OldCompBot.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
        public static final double maxAngularRate = RotationsPerSecond.of(15).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

        public static final double headingKP = 5.0;
        public static final double headingKI = 0.0;
        public static final double headingKD = 0.0;

        public static final double translationKP = 3.0;
        public static final double translationKI = 0.0;
        public static final double translationKD = 0.0;
    }

    public static enum RobotType {
        //sim
        SIM,
        //real
        REAL,
        //replay
        REPLAY,
        // woodbot
        WOODBOT,
        // practice bot
        PRACTICE,
        // comp bot
        COMPETITION,
        // last year's comp bot; abbreviated to OCB
        OLD_COMP_BOT,
    }

    public static final class SerialAddressConstants {
        public static String OCB_SERIAL_ADDRESS = "032BE44A";
        public static String WOOD_SERIAL_ADDRESS = "b";
        public static String PRACTICE_SERIAL_ADDRESS = "c";
        public static String COMP_SERIAL_ADDRESS = "d";
    }

    public static RobotType getRobotType() {
        String serialAddress = HALUtil.getSerialNumber();

        if (serialAddress.equals(SerialAddressConstants.PRACTICE_SERIAL_ADDRESS)) {
            return Constants.RobotType.PRACTICE;
        } else if (serialAddress.equals(SerialAddressConstants.COMP_SERIAL_ADDRESS)) {
            return Constants.RobotType.COMPETITION;
        } else if (serialAddress.equals(SerialAddressConstants.WOOD_SERIAL_ADDRESS)) {
            return Constants.RobotType.WOODBOT;
        } else if (serialAddress.equals(SerialAddressConstants.OCB_SERIAL_ADDRESS)) {
            return Constants.RobotType.OLD_COMP_BOT;
        } else if (!Robot.isReal()) { // KEEP AT BOTTOM
          return Constants.RobotType.SIM;
        }

        return Constants.RobotType.COMPETITION;
    }

    public static boolean isOCB() {
        if (getRobotType() == RobotType.OLD_COMP_BOT) {
            return true;
        }
        return false;
    }

    public static boolean isWoodBot() {
        if (getRobotType() == RobotType.WOODBOT) {
            return true;
        }
        return false;
    }

    public static boolean isPracticeBot() {
        if (getRobotType() == RobotType.PRACTICE) {
            return true;
        }
        return false;
    }

    public static boolean isCompBot() {
        if (getRobotType() == RobotType.COMPETITION) {
            return true;
        }
        return false;
    }
}
