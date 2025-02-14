package frc;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.commands.AlignWithLimelight;

import static org.junit.jupiter.api.Assertions.assertEquals;

import edu.wpi.first.hal.HAL;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

public class RunCodeTest {

    @Test
    public void rotatePIDSpeedsByNeg120() {
        runRotationTest(1.0, 1.0, -120.0);
    }

    @Test
    public void rotatePIDSpeedsByNeg60() {
        runRotationTest(1.0, 1.0, -60.0);
    }

    @Test
    public void rotatePIDSpeedsBy0() {
        Translation2d baseSpeeds = new Translation2d(1.0, 1.0);
        Rotation2d rotation = Rotation2d.fromDegrees(0.0);

        Translation2d rotatedSpeeds = AlignWithLimelight.rotateTranslation(baseSpeeds, rotation);
        assertEquals(1.0, rotatedSpeeds.getX(), "X component of translation should match");
        assertEquals(1.0, rotatedSpeeds.getY(), "Y component of translation should match");
    }

    @Test
    public void rotatePIDSpeedsBy60() {
        runRotationTest(1.0, 1.0, 60.0);
    }

    @Test
    public void rotatePIDSpeedsBy120() {
        runRotationTest(1.0, 1.0, 120.0);
    }

    @Test
    public void rotatePIDSpeedsBy180() {
        runRotationTest(1.0, 1.0, 180.0);
    }


    private void runRotationTest(double x, double y, double angleDegrees){
        double angleRadians = Math.toRadians(angleDegrees);
        Translation2d baseSpeeds = new Translation2d(x, y);
        Rotation2d rotation = Rotation2d.fromDegrees(angleDegrees);

        Translation2d rotatedSpeeds = AlignWithLimelight.rotateTranslation(baseSpeeds, rotation);
        assertEquals(x * Math.cos(angleRadians) - y * Math.sin(angleRadians), rotatedSpeeds.getX(), "X component of translation should match");
        assertEquals(x * Math.sin(angleRadians) + y * Math.cos(angleRadians), rotatedSpeeds.getY(), "Y component of translation should match");
    }
}
