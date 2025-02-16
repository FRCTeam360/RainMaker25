// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.signals.ConnectedMotorValue;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
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
    public static final AprilTagFieldLayout FIELD_LAYOUT = AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape);

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
    OLD_COMP_BOT
  }

  public static final class SerialAddressConstants {
    public static String OCB_SERIAL_ADDRESS = "";
    public static String WOOD_SERIAL_ADDRESS = "031b5208";
    public static String PRACTICE_SERIAL_ADDRESS = "03260AD5";
    public static String COMP_SERIAL_ADDRESS = "d";
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
        public static final double WBGOALSCORETY = -10.0;

        public static final int ELEVATOR_BOTTOM_SWITCH = 2;
        public static final int OUTTAKE_SENSOR = 0; //DIO port
    }

    public static final class PracticeBotConstants {
        public static final int ALGAE_SHOOTER_ID = 4;
        public static final int ALGAE_ARM_ID = 5;
        public static final int CLIMBER_ID = 6;
        public static final int CORAL_SHOOTER_ID = 19;
        public static final int INTAKE_SENSOR_ID = 20;
        public static final int OUTTAKE_SENSOR_ID = 21;
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