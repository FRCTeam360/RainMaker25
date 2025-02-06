package frc.robot.subsystems.Vision;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

public record VisionMeasurement(
        double timestamp,
        Pose2d estimatedPose,
        Matrix<N3, N1> standardDeviation) {
}
