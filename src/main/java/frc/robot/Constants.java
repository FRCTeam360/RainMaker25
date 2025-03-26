// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.hal.HALUtil;
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
    public final class SetPointConstants{

        public static final double RIGHT_GOAL_TY = isCompBot() ? 14.5 : 11.75; //TUNED FOR GPEAK
        public static final double RIGHT_GOAL_TX = 0.0;

        public static final double LEFT_GOAL_TY = RIGHT_GOAL_TY;
        public static final double LEFT_GOAL_TX = 0.0;

        public class ElevatorHeights {
            public static final double TELE_LEVEL_FOUR = isCompBot() ? 29.5 - 0.3 : 29.5; //added 0.2 3/20
            public static final double TELE_LEVEL_THREE = isCompBot() ? 16.1 : 16.0;
            public static final double TELE_LEVEL_TWO = isCompBot() ? 7.4 : 7.0;
            public static final double TELE_LEVEL_ONE = 0.0;

            public static final double AUTO_LEVEL_FOUR = isCompBot() ? 29.0 + 0.2 : 29.5;
            public static final double AUTO_LEVEL_THREE = 20.5;
            public static final double AUTO_LEVEL_TWO = 0.0;
            public static final double AUTO_LEVEL_ONE = 0.0;

            public static final double ALIGN_LEVEL_FOUR = 0.0;
            public static final double ALIGN_LEVEL_THREE = 0.0;
            public static final double ALIGN_LEVEL_TWO = 0.0;
            public static final double ALIGN_LEVEL_ONE = 0.0;
        } 
    }
    public static final AprilTagFieldLayout FIELD_LAYOUT = AprilTagFieldLayout.loadField(
        AprilTagFields.k2025ReefscapeWelded
    );

   
    public static class VisionConstants {
        public static final String WOODBOT_LIMELIGHT_NAME = "limelight";

        public static final String OCB_LIMELIGHT_NAME = "limelight";
    }

    // public static final Mode currentMode = Mode.SIM;

    public static enum RobotType {
        // real robot
        REAL,
        // physics sim
        SIM,
        // log file
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
        public static String OCB_SERIAL_ADDRESS = "DO_NOT_USE";
        public static String WOOD_SERIAL_ADDRESS = "DO_NOT_USE";
        public static String PRACTICE_SERIAL_ADDRESS = "03260AD5";
        public static String COMP_SERIAL_ADDRESS = "03415A99";
    }

    // placeholders for now until we got woodbot working
    public static final int CORAL_INTAKE_ID = 0;
    public static final int CORAL_OUTTAKE_ID = 1;
    public static final int ELEVATOR_ID = 2;

    public static class OperatorConstants {
        public static final int kDriverControllerPort = 0;
    }

    public static class OldCompBotConstants {
        public static final String OCB_LIMELIGHT_NAME = "limelight";
    }

    public static final class WoodbotConstants {
        public static final int CORAL_INTAKE_ID = 25;
        public static final int CORAL_SHOOTER_ID = 20;
        public static final int ELEVATOR_ID = 14;

        public static final double headingKP = 4.0;
        public static final double headingKI = 0.0;
        public static final double headingKD = 0.0;
        public static final double headingKIZone = 0.0;

        public static final double WBGOALSCORETX = 0.0;
        public static final double WBGOALSCORETY = -11.0;

        public static final int ELEVATOR_BOTTOM_SWITCH = 2;
        public static final int OUTTAKE_SENSOR = 0; //DIO port

        public static final String CORAL_LIMELIGHT_NAME = "limelight-coral";
        public static final String ALGAE_LIMELIGHT_NAME = "limelight-algae";
    }

    public static final class PracticeBotConstants {
        public static final String CANBUS_NAME = "Default Name";

        public static final int SERVO_PORT = 0; // is the actual port :)
        
        public static final int BACK_ELEVATOR_ID = 14;
        public static final int FRONT_ELEVATOR_ID = 15;

        public static final int CLIMBER_ROLLER_ID = 28;
        public static final int CLIMBER_WINCH_ID = 17;

        public static final int ALGAE_ARM_ID = 18;
        public static final int CORAL_SHOOTER_ID = 19;

        public static final int ALGAE_SHOOTER_FRONT_ID = 25;
        public static final int ALGAE_SHOOTER_BACK_ID = 26;
        public static final int ALGAE_TILT = 27;
        public static final int ALGAE_ROLLER = 16;

        public static final int INTAKE_SENSOR_ID = 20;
        public static final int OUTTAKE_SENSOR_ID = 21;

        public static final double RIGHT_GOAL_TY = 12.0; //praccy bot 3/15
        public static final double RIGHT_GOAL_TX = 0.0;

        public static final double LEFT_GOAL_TY = RIGHT_GOAL_TY;
        public static final double LEFT_GOAL_TX = 0;

        public static final String CORAL_LIMELIGHT_NAME = "limelight-coral";
        public static final String ALGAE_LIMELIGHT_NAME = "limelight-algae";

    }

    public static final class CompBotConstants { // Currently just a copy of practice bot, values should be adjusted to comp if needed
        public static final String CANBUS_NAME = "Default Name";

        public static final int SERVO_PORT = 0; // is the actual port :)
        public static final int BACK_ELEVATOR_ID = 14;
        public static final int FRONT_ELEVATOR_ID = 15;

        public static final int CLIMBER_ROLLER_ID = 28;
        public static final int CLIMBER_WINCH_ID = 16;

        public static final int CORAL_SHOOTER_ID = 17;
        public static final int ALGAE_ARM_ID = 18;
        
        public static final int INTAKE_SENSOR_ID = 19;
        public static final int OUTTAKE_SENSOR_ID = 20;

        public static final int ALGAE_SHOOTER_FRONT_ID = 21;
        public static final int ALGAE_SHOOTER_BACK_ID = 22;
        public static final int ALGAE_ROLLER = 23;
        public static final int ALGAE_TILT = 24;


        public static final double TELE_RIGHT_GOAL_TY = 13.75; //practice field: 15.5 | gpeak: 14.0 | auburn: 13.75
        public static final double TELE_RIGHT_GOAL_TX = 0.0;
        public static final double TELE_LEFT_GOAL_TY = TELE_RIGHT_GOAL_TY;
        public static final double TELE_LEFT_GOAL_TX = 0.0;
        
        public static final double AUTO_RIGHT_GOAL_TY = 13.75; 
        public static final double AUTO_RIGHT_GOAL_TX = 0.0;
        public static final double AUTO_LEFT_GOAL_TY = AUTO_RIGHT_GOAL_TY;
        public static final double AUTO_LEFT_GOAL_TX = 0.0;

        public static final String CORAL_LIMELIGHT_NAME = "limelight-coral";
        public static final String ALGAE_LIMELIGHT_NAME = "limelight-algae";

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

    public static boolean isSim() {
        if (getRobotType() == RobotType.SIM) {
            return true;
        }
        return false;
    }
}
