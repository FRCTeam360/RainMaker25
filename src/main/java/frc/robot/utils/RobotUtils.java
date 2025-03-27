package frc.robot.utils;

import java.io.File;

import edu.wpi.first.math.geometry.Rotation2d;

public class RobotUtils {
    public static boolean isUsbWriteable() {
        File usb = new File("/U");
        if (usb.exists() && usb.isDirectory()) {
            try {

                File temporaryFile = File.createTempFile("usb", ".txt", usb);
                temporaryFile.delete();
                return true;
            } catch (Exception e) {
                System.out.println("usb not found");

            }
        }
        return false;
    }

    public static Rotation2d flipForRedAlliancePerspective(Rotation2d rotation2d) {
        double angle = rotation2d.getDegrees();
        if (angle <= 0.0) {
            angle = angle + 180.0;
        } else {
            angle = angle - 180.0;
        }
        return Rotation2d.fromDegrees(angle);
    }
}
