package frc.robot.commands;

import java.util.ArrayList;
import java.util.Collections;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Map.Entry;
import java.util.Set;
import java.util.function.Supplier;
import java.util.stream.Collectors;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Vision.Vision;
import frc.robot.utils.CommandLogger;

public class PathOnTheFly {
    public static class ReefPositions {
        public static final Map<Integer, Pose2d> LEFT_TAG_ID_TO_POSITION = Map.ofEntries(

                // ADD SELECT COMMAND AND COLLISION AVOIDENCE STUFFFF
                Map.entry(
                        6,
                        new Pose2d(13.628, 2.612, Rotation2d.fromDegrees(120.0))),
                Map.entry(
                        7,
                        new Pose2d(14.623, 3.809, Rotation2d.fromDegrees(180.0))),
                Map.entry(
                        8,
                        new Pose2d(14.176, 5.236, Rotation2d.fromDegrees(-120.0))),
                Map.entry(
                        9,
                        new Pose2d(12.503, 5.453, Rotation2d.fromDegrees(-60.0))),
                Map.entry(
                        10,
                        new Pose2d(11.522, 4.371, Rotation2d.fromDegrees(0.0))),
                Map.entry(
                        11,
                        new Pose2d(11.955, 2.871, Rotation2d.fromDegrees(60.0))),
                Map.entry(
                        17,
                        new Pose2d(3.725, 3.011, Rotation2d.fromDegrees(60.0))),
                Map.entry(
                        18,
                        new Pose2d(3.247, 4.181, Rotation2d.fromDegrees(0.0))),
                Map.entry(
                        19,
                        new Pose2d(3.832, 5.482, Rotation2d.fromDegrees(-60.0))),
                Map.entry(
                        20,
                        new Pose2d(5.529, 5.167, Rotation2d.fromDegrees(-120.0))),
                Map.entry(
                        21,
                        new Pose2d(5.743, 3.840, Rotation2d.fromDegrees(180.0))),
                Map.entry(
                        22,
                        new Pose2d(5.529, 2.778, Rotation2d.fromDegrees(120.0))));
        // RIGHT STARTS HERE
        public static final Map<Integer, Pose2d> RIGHT_TAG_ID_TO_POSITION = Map.ofEntries(
                Map.entry(
                        6,
                        new Pose2d(14.219, 2.828, Rotation2d.fromDegrees(120.0))),
                Map.entry(
                        7,
                        new Pose2d(14.536, 4.386, Rotation2d.fromDegrees(180.0))),
                Map.entry(
                        8,
                        new Pose2d(13.642, 5.481, Rotation2d.fromDegrees(-120.0))),
                Map.entry(
                        9,
                        new Pose2d(12.027, 5.207, Rotation2d.fromDegrees(-60.0))),
                Map.entry(
                        10,
                        new Pose2d(11.565, 3.679, Rotation2d.fromDegrees(0.0))),
                Map.entry(
                        11,
                        new Pose2d(12.474, 2.597, Rotation2d.fromDegrees(60.0))),
                Map.entry(
                        17,
                        new Pose2d(4.046, 2.865, Rotation2d.fromDegrees(60.0))),
                Map.entry(
                        18,
                        new Pose2d(3.276, 3.859, Rotation2d.fromDegrees(0.0))),
                Map.entry(
                        19,
                        new Pose2d(3.381, 5.212, Rotation2d.fromDegrees(-60.0))),
                Map.entry(
                        20,
                        new Pose2d(5.109, 5.467, Rotation2d.fromDegrees(-120.0))),
                Map.entry(
                        21,
                        new Pose2d(5.743, 4.132, Rotation2d.fromDegrees(180.0))),
                Map.entry(
                        22,
                        new Pose2d(5.529, 2.778, Rotation2d.fromDegrees(120.0))));

        public static final Map<Integer, Double> REEF_TAG_ID_TO_ROTATION = Map.ofEntries(
                Map.entry(6, 120.0),
                Map.entry(7, 180.0),
                Map.entry(8, -120.0),
                Map.entry(9, -60.0),
                Map.entry(10, 0.0),
                Map.entry(11, 60.0),
                Map.entry(17, 60.0),
                Map.entry(18, 0.0),
                Map.entry(19, -60.0),
                Map.entry(20, -120.0),
                Map.entry(21, 180.0),
                Map.entry(22, 120.0));

        private static final Set<Integer> REEF_TAG_IDS_RED = Set.of(6, 7, 8, 9, 10, 11);
        private static final Set<Integer> REEF_TAG_IDS_BLUE = Set.of(17, 18, 19, 20, 21, 22);
        private static final Set<Integer> REEF_TAG_IDS = Set.of(6, 7, 8, 9, 10, 11, 17, 18, 19, 20, 21, 22);

        private static final List<AprilTag> REEF_TAGS = Constants.FIELD_LAYOUT.getTags().stream()
                .filter(tag -> REEF_TAG_IDS.contains(tag.ID))
                .collect(Collectors.collectingAndThen(Collectors.toList(), Collections::unmodifiableList));
        private static final List<AprilTag> REEF_TAGS_RED = Constants.FIELD_LAYOUT.getTags().stream()
                .filter(tag -> REEF_TAG_IDS_RED.contains(tag.ID))
                .collect(Collectors.collectingAndThen(Collectors.toList(), Collections::unmodifiableList));
        private static final List<AprilTag> REEF_TAGS_BLUE = Constants.FIELD_LAYOUT.getTags().stream()
                .filter(tag -> REEF_TAG_IDS_BLUE.contains(tag.ID))
                .collect(Collectors.collectingAndThen(Collectors.toList(), Collections::unmodifiableList));

        private static final Map<Integer, Pose2d> LEFT_TAG_ID_TO_POSITION_DYNAMIC = REEF_TAGS.stream()
                .map(tag -> createTagScoringPositionEntry(tag, false))
                .collect(Collectors.toMap(
                        Map.Entry::getKey,
                        Map.Entry::getValue));

        private static final Map<Integer, Pose2d> RIGHT_TAG_ID_TO_POSITION_DYNAMIC = REEF_TAGS.stream()
                .map(tag -> createTagScoringPositionEntry(tag, true))
                .collect(Collectors.toMap(
                        Map.Entry::getKey,
                        Map.Entry::getValue));

        private static Entry<Integer, Pose2d> createTagScoringPositionEntry(AprilTag tag, boolean right) {
            return Map.entry(tag.ID, transformTagPoseToScoringPose(tag, right));
        }

        private static Pose2d transformTagPoseToScoringPose(AprilTag tag, boolean right) {
            Pose2d tagPose = tag.pose.toPose2d();
            Rotation2d tagRotation = tagPose.getRotation();

            if (right) {
                // x is forwards-backwards, y is right-left relative to the tag position
                Translation2d tagTranslation = new Translation2d(0.45, 0.14);
                tagTranslation.rotateBy(tagRotation);
                return tagPose.plus(new Transform2d(tagTranslation, Rotation2d.k180deg));
            } else {
                // x is forwards-backwards, y is right-left relative to the tag position
                Translation2d tagTranslation = new Translation2d(0.45, -0.14);
                tagTranslation.rotateBy(tagRotation);
                return tagPose.plus(new Transform2d(tagTranslation, Rotation2d.k180deg));
            }
        }
    }

    /**
     * Follow the path given in the waypoints list, ending at the desired rotation
     * 
     * @param wayPoints
     * @param endRotation2d
     * @return
     */
    public static Command pathOnTheFly(List<Waypoint> wayPoints, Rotation2d endRotation2d) {
        PathConstraints constraints = new PathConstraints(3.0, 3.0, 2 * Math.PI, 4 * Math.PI); // test constraints(to be
                                                                                               // changed)

        PathPlannerPath path = new PathPlannerPath(
                wayPoints,
                constraints,
                null,
                new GoalEndState(0.0, endRotation2d));

        path.preventFlipping = true;

        return AutoBuilder.followPath(path);
    }

    /**
     * Move to the given position using Path Planner's pathfinding functionality
     * 
     * @param endPose
     * @return
     */
    public static Command pathFindToPose(Pose2d endPose, CommandSwerveDrivetrain drivetrain) {
        // test constraint
        PathConstraints constraints = new PathConstraints(3.0, 3.0, 2 * Math.PI, 4 * Math.PI);
        return CommandLogger.logCommand(AutoBuilder.pathfindToPose(endPose, constraints), "PathFind")
                        .andThen(FaceAngle.getCommand(drivetrain, endPose.getRotation().getDegrees()));
    }

    /**
     * Moves to the closest reef scoring position on the robot's alliance side
     * 
     * @param currentBotPose supplier for the robot's current position
     * @param right          if true, move to the right pole of the reef. If false,
     *                       move to the left pole
     * @return Command to path find to the reef
     */
    public static Command pathfindToReef(Supplier<Pose2d> currentBotPose, boolean right, CommandSwerveDrivetrain drivetrain) {
        return Commands.either(
                pathfindToReefHelper(currentBotPose, ReefPositions.RIGHT_TAG_ID_TO_POSITION_DYNAMIC, drivetrain),
                pathfindToReefHelper(currentBotPose, ReefPositions.LEFT_TAG_ID_TO_POSITION_DYNAMIC, drivetrain),
                () -> right);
    }

    /**
     * Helper method to fill out the select commands for the left and right sides of
     * the reef
     * 
     * @param currentBotPose
     * @param tagIDPositionMap
     * @return
     */
    private static Command pathfindToReefHelper(Supplier<Pose2d> currentBotPose,
            Map<Integer, Pose2d> tagIDPositionMap, CommandSwerveDrivetrain drivetrain) {
        return Commands.select(buildPathfindToReefCommandMap(tagIDPositionMap, drivetrain),
                () -> getNearestReefTagID(currentBotPose.get()));
    }

    private static Map<Integer, Command> buildPathfindToReefCommandMap(Map<Integer, Pose2d> tagIDPositionMap, CommandSwerveDrivetrain drivetrain) {
        return ReefPositions.REEF_TAG_IDS.stream()
                .map(tagId -> buildPathfindToReefCommandEntry(tagId, tagIDPositionMap, drivetrain))
                .collect(Collectors.toMap(
                        Map.Entry::getKey,
                        Map.Entry::getValue));
    }

    private static Entry<Integer, Command> buildPathfindToReefCommandEntry(int tagID,
            Map<Integer, Pose2d> tagIDPositionMap, CommandSwerveDrivetrain drivetrain) {
        Pose2d endPose = tagIDPositionMap.get(tagID);
        return Map.entry(tagID, pathFindToPose(endPose, drivetrain));
    }

    private static int getNearestReefTagID(Pose2d currentBotPose) {
        List<AprilTag> tags = ReefPositions.REEF_TAGS;
        // If red alliance
        if (DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red) {
            tags = ReefPositions.REEF_TAGS_RED;
            // If blue alliance
        } else if (DriverStation.getAlliance().isPresent()) {
            tags = ReefPositions.REEF_TAGS_BLUE;
        }
        return getNearestTagID(currentBotPose, tags);
    }

    private static int getNearestTagID(Pose2d currentBotPose, List<AprilTag> tags) {
        AprilTag closestTag = tags.get(0);
        double minDist = Double.MAX_VALUE;
        for (AprilTag tag : tags) {
            double distance = currentBotPose.getTranslation().getDistance(tag.pose.toPose2d().getTranslation());
            if (distance < minDist) {
                closestTag = tag;
                minDist = distance;
            }
        }
        return closestTag.ID;
    }
}
