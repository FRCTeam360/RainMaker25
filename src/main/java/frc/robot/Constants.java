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

        public static final double headingKP = 4.0;
        public static final double headingKI = 0.0;
        public static final double headingKD = 0.0;
        public static final double headingKIZone = 0.0;

        public static final double translationKP = 0.5;
        public static final double translationKI = 0.0;
        public static final double translationKD = 0.0;
    }

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
// 一䔀䌀吀䔀䐀 一吀㐀 挀氀椀攀渀琀 ✀䄀搀瘀愀渀琀愀最攀匀挀漀瀀攀䀀㈀✀ ⠀昀爀漀洀 ㄀　⸀㌀⸀㘀　⸀㈀　㄀㨀㔀　㔀㔀㈀⤀ ＀෾਀＀￾￾￾￾￾￾￾埾愀爀渀椀渀最 ＀￾⃾㐀㐀　　㐀 ＀￾⃾䘀刀䌀㨀 吀栀攀 䐀爀椀瘀攀爀 匀琀愀琀椀漀渀 栀愀猀 氀漀猀琀 挀漀洀洀甀渀椀挀愀琀椀漀渀 眀椀琀栀 琀栀攀 爀漀戀漀琀⸀ ＀￾⃾䐀爀椀瘀攀爀 匀琀愀琀椀漀渀 ＀￾￾෾਀＀￾￾￾￾￾⃾一吀㨀 䜀漀琀 愀 一吀㐀 挀漀渀渀攀挀琀椀漀渀 昀爀漀洀 ㄀　⸀㌀⸀㘀　⸀㈀　㄀ 瀀漀爀琀 㔀　㔀㔀㌀ ＀෾਀＀￾￾￾￾￾￾￾埾愀爀渀椀渀最 ＀￾⃾㐀㐀　　㐀 ＀￾⃾䘀刀䌀㨀 吀栀攀 䐀爀椀瘀攀爀 匀琀愀琀椀漀渀 栀愀猀 氀漀猀琀 挀漀洀洀甀渀椀挀愀琀椀漀渀 眀椀琀栀 琀栀攀 爀漀戀漀琀⸀ ＀￾⃾䐀爀椀瘀攀爀 匀琀愀琀椀漀渀 ＀￾￾෾਀＀￾￾￾￾￾⃾一吀㨀 䌀伀一一䔀䌀吀䔀䐀 刀吀吀 挀氀椀攀渀琀 ⠀昀爀漀洀 ㄀　⸀㌀⸀㘀　⸀㈀　㄀㨀㔀　㔀㔀㌀⤀ ＀෾਀＀￾￾￾￾￾⃾一吀㨀 䜀漀琀 愀 一吀㐀 挀漀渀渀攀挀琀椀漀渀 昀爀漀洀 ㄀　⸀㌀⸀㘀　⸀㈀　㄀ 瀀漀爀琀 㔀　㔀㌀　 ＀෾਀＀￾￾￾￾￾⃾一吀㨀 䜀漀琀 愀 一吀㐀 挀漀渀渀攀挀琀椀漀渀 昀爀漀洀 ㄀　⸀㌀⸀㘀　⸀㈀　㄀ 瀀漀爀琀 㔀　㔀㌀㘀 ＀෾਀＀￾￾￾￾￾⃾一吀㨀 䜀漀琀 愀 一吀㐀 挀漀渀渀攀挀琀椀漀渀 昀爀漀洀 ㄀　⸀㌀⸀㘀　⸀㈀　㄀ 瀀漀爀琀 㔀　㔀㌀㄀ ＀෾਀＀￾￾￾￾￾⃾一吀㨀 䐀䤀匀䌀伀一一䔀䌀吀䔀䐀 刀吀吀 挀氀椀攀渀琀 ⠀昀爀漀洀 ㄀　⸀㌀⸀㘀　⸀㈀　㄀㨀㔀　㔀㄀㈀⤀㨀 爀攀洀漀琀攀 挀氀漀猀攀㨀  ＀෾਀＀￾￾￾￾￾⃾一吀㨀 䜀漀琀 愀 一吀㐀 挀漀渀渀攀挀琀椀漀渀 昀爀漀洀 ㄀　⸀㌀⸀㘀　⸀㈀　㄀ 瀀漀爀琀 㔀　㔀㜀㔀 ＀෾਀＀￾￾￾￾￾￾￾埾愀爀渀椀渀最 ＀￾⃾㐀㐀　　㐀 ＀￾⃾䘀刀䌀㨀 吀栀攀 䐀爀椀瘀攀爀 匀琀愀琀椀漀渀 栀愀猀 氀漀猀琀 挀漀洀洀甀渀椀挀愀琀椀漀渀 眀椀琀栀 琀栀攀 爀漀戀漀琀⸀ ＀￾⃾䐀爀椀瘀攀爀 匀琀愀琀椀漀渀 ＀￾￾෾਀＀￾￾￾￾￾⃾一吀㨀 䌀伀一一䔀䌀吀䔀䐀 一吀㐀 挀氀椀攀渀琀 ✀猀栀甀昀昀氀攀戀漀愀爀搀䀀㌀✀ ⠀昀爀漀洀 ㄀　⸀㌀⸀㘀　⸀㈀　㄀㨀㔀　㔀㜀㔀⤀ ＀෾਀＀￾￾￾￾￾￾￾埾愀爀渀椀渀最 ＀￾⃾㐀㐀　　㐀 ＀￾⃾䘀刀䌀㨀 吀栀攀 䐀爀椀瘀攀爀 匀琀愀琀椀漀渀 栀愀猀 氀漀猀琀 挀漀洀洀甀渀椀挀愀琀椀漀渀 眀椀琀栀 琀栀攀 爀漀戀漀琀⸀ ＀￾⃾䐀爀椀瘀攀爀 匀琀愀琀椀漀渀 ＀￾￾෾਀＀￾￾￾￾￾￾￾埾愀爀渀椀渀最 ＀￾⃾㐀㐀　　㐀 ＀￾⃾䘀刀䌀㨀 吀栀攀 䐀爀椀瘀攀爀 匀琀愀琀椀漀渀 栀愀猀 氀漀猀琀 挀漀洀洀甀渀椀挀愀琀椀漀渀 眀椀琀栀 琀栀攀 爀漀戀漀琀⸀ ＀￾⃾䐀爀椀瘀攀爀 匀琀愀琀椀漀渀 ＀￾￾෾਀＀￾￾￾￾￾￾￾埾愀爀渀椀渀最 ＀￾⃾㐀㐀　　㐀 ＀￾⃾䘀刀䌀㨀 吀栀攀 䐀爀椀瘀攀爀 匀琀愀琀椀漀渀 栀愀猀 氀漀猀琀 挀漀洀洀甀渀椀挀愀琀椀漀渀 眀椀琀栀 琀栀攀 爀漀戀漀琀⸀ ＀￾⃾䐀爀椀瘀攀爀 匀琀愀琀椀漀渀 ＀￾￾෾਀＀￾￾￾￾￾￾￾埾愀爀渀椀渀最 ＀￾⃾㐀㐀　　㐀 ＀￾⃾䘀刀䌀㨀 吀栀攀 䐀爀椀瘀攀爀 匀琀愀琀椀漀渀 栀愀猀 氀漀猀琀 挀漀洀洀甀渀椀挀愀琀椀漀渀 眀椀琀栀 琀栀攀 
// GOOD﻿,  FMS-bad﻿﻿ Driver Station ﻿﻿﻿
// ﻿﻿﻿﻿﻿﻿﻿﻿Warning ﻿﻿ 44004 ﻿﻿ FRC: The Driver Station has lost communication with the robot. ﻿﻿ Driver Station ﻿﻿﻿
// ﻿﻿﻿﻿﻿﻿﻿﻿Warning ﻿﻿ 44004 ﻿﻿ FRC: The Driver Station has lost communication with the robot. ﻿﻿ Driver Station ﻿﻿﻿
// ﻿﻿﻿﻿﻿﻿﻿﻿Warning ﻿﻿ 44004 ﻿﻿ FRC: The Driver Station has lost communication with the robot. ﻿﻿ Driver Station ﻿﻿﻿
// ﻿﻿﻿﻿﻿﻿﻿﻿Warning ﻿﻿ 44004 ﻿﻿ FRC: The Driver Station has lost communication with the robot. ﻿﻿ Driver Station ﻿﻿﻿
// ﻿﻿﻿﻿﻿﻿﻿﻿Warning ﻿﻿ 44004 ﻿﻿ FRC: The Driver Station has lost communication with the robot. ﻿﻿ Driver Station ﻿﻿﻿
// ﻿﻿﻿﻿﻿﻿﻿﻿Warning ﻿﻿ 44004 ﻿﻿ FRC: The Driver Station has lost communication with the robot. ﻿﻿ Driver Station ﻿﻿﻿
// ﻿﻿﻿﻿﻿﻿﻿﻿Warning ﻿﻿ 44004 ﻿﻿ FRC: The Driver Station has lost communication with the robot. ﻿﻿ Driver Station ﻿﻿﻿
// ﻿﻿﻿﻿﻿﻿﻿﻿Warning ﻿﻿ 44004 ﻿﻿ FRC: The Driver Station has lost communication with the robot. ﻿﻿ Driver Station ﻿﻿﻿
// ﻿﻿﻿﻿﻿﻿﻿﻿Warning ﻿﻿ 44004 ﻿﻿ FRC: The Driver Station has lost communication with the robot. ﻿﻿ Driver Station ﻿﻿﻿
// ﻿﻿﻿﻿﻿﻿﻿﻿Warning ﻿﻿ 44004 ﻿﻿ FRC: The Driver Station has lost communication with the robot. ﻿﻿ Driver Station ﻿﻿﻿
// ﻿﻿﻿﻿﻿﻿﻿﻿Warning ﻿﻿ 44004 ﻿﻿ FRC: The Driver Station has lost communication with the robot. ﻿﻿ Driver Station ﻿﻿﻿
// ﻿﻿﻿﻿﻿﻿﻿﻿Warning ﻿﻿ 44004 ﻿﻿ FRC: The Driver Station has lost communication with the robot. ﻿﻿ Driver Station ﻿﻿﻿
// ﻿﻿﻿﻿﻿﻿﻿﻿Warning ﻿﻿ 44000 ﻿﻿ Driver Station not keeping up with protocol rates ﻿﻿ Driver Station ﻿﻿﻿
// ﻿﻿﻿﻿﻿﻿﻿﻿Warning ﻿﻿ 44002 ﻿﻿ Ping Results:  ﻿link-bad﻿,  DS radio(.4)-bad﻿,  robot radio(.1)-bad﻿,  roboRIO(.2)-bad﻿,  FMS-bad﻿﻿ Driver Station ﻿﻿﻿
