package frc.robot.commands;

import java.util.Collections;
import java.util.List;
import java.util.Map;
import java.util.Map.Entry;
import java.util.Set;
import java.util.function.Supplier;
import java.util.stream.Collectors;

import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.utils.CommandLogger;

public class PathOnTheFly {
    private static final String LOGGING_PREFIX = "PathOnTheFly: ";

    public static Command pathfindToProcessor(CommandSwerveDrivetrain drivetrain){
        final int processorTagIDBlue = 16;
        final int processorTagIDRed = 3;

        final Pose2d processorPoseBlue = Constants.FIELD_LAYOUT.getTagPose(processorTagIDBlue).get().toPose2d().plus(new Transform2d(0.5, 0.0,Rotation2d.kCCW_90deg));
        final Pose2d processorPoseRed = Constants.FIELD_LAYOUT.getTagPose(processorTagIDRed).get().toPose2d().plus(new Transform2d(0.5, 0.0,Rotation2d.kCCW_90deg));

        Logger.recordOutput(LOGGING_PREFIX + "Processor: Blue", processorPoseBlue);
        Logger.recordOutput(LOGGING_PREFIX + "Processor: Red", processorPoseRed);

        return Commands.either(
                pathfindToPose(drivetrain, processorPoseRed),
                pathfindToPose(drivetrain, processorPoseBlue),
                () -> DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red
        );
    }

    /**
     * Move to the given position using Path Planner's pathfinding functionality
     *
     * @param drivetrain
     * @param endPose
     * @return
     */
    public static Command pathfindToPose(CommandSwerveDrivetrain drivetrain, Pose2d endPose) {
        // test constraint
        PathConstraints constraints = new PathConstraints(3.0, 3.0, 2 * Math.PI, 4 * Math.PI);
        return CommandLogger.logCommand(AutoBuilder.pathfindToPose(endPose, constraints), "PathFind")
            .andThen(DriveToPose.getCommand(drivetrain, endPose));
    }
}
