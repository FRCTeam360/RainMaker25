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
    public static class ReefPositions {
        public static final Map<Integer, Translation2d> LEFT_TAG_ID_TO_POSITION_OFFSET = Map.ofEntries(
                // ADD SELECT COMMAND AND COLLISION AVOIDENCE STUFFFF
                Map.entry(
                        6,
                        new Translation2d(0, 0)),
                Map.entry(
                        7,
                        new Translation2d(0, 0)),
                Map.entry(
                        8,
                        new Translation2d(0, 0)),
                Map.entry(
                        9,
                        new Translation2d(0, 0)),
                Map.entry(
                        10,
                        new Translation2d(0, 0)),
                Map.entry(
                        11,
                        new Translation2d(0, 0)),
                Map.entry(
                        17,
                        new Translation2d(0, 0)),
                Map.entry(
                        18,
                        new Translation2d(0, 0)),
                Map.entry(
                        19,
                        new Translation2d(0, 0)),
                Map.entry(
                        20,
                        new Translation2d(0, 0)),
                Map.entry(
                        21,
                        new Translation2d(0, 0)),
                Map.entry(
                        22,
                        new Translation2d(0, 0)));
        // RIGHT STARTS HERE
        public static final Map<Integer, Translation2d> RIGHT_TAG_ID_TO_POSITION = Map.ofEntries(

                // ADD SELECT COMMAND AND COLLISION AVOIDENCE STUFFFF
                Map.entry(
                        6,
                        new Translation2d(0, 0)),
                Map.entry(
                        7,
                        new Translation2d(0, 0)),
                Map.entry(
                        8,
                        new Translation2d(0, 0)),
                Map.entry(
                        9,
                        new Translation2d(0, 0)),
                Map.entry(
                        10,
                        new Translation2d(0, 0)),
                Map.entry(
                        11,
                        new Translation2d(0, 0)),
                Map.entry(
                        17,
                        new Translation2d(0, 0)),
                Map.entry(
                        18,
                        new Translation2d(0, 0)),
                Map.entry(
                        19,
                        new Translation2d(0, 0)),
                Map.entry(
                        20,
                        new Translation2d(0, 0)),
                Map.entry(
                        21,
                        new Translation2d(0, 0)),
                Map.entry(
                        22,
                        new Translation2d(0, 0)));

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
                Translation2d offset = RIGHT_TAG_ID_TO_POSITION.get(tag.ID);
                // x is forwards-backwards, y is right-left relative to the tag position
                Translation2d tagTranslation = new Translation2d(0.45, 0.14).plus(offset);
                tagTranslation.rotateBy(tagRotation);
                return tagPose.plus(new Transform2d(tagTranslation, Rotation2d.k180deg));
            } else {
                Translation2d offset = LEFT_TAG_ID_TO_POSITION_OFFSET.get(tag.ID);
                // x is forwards-backwards, y is right-left relative to the tag position
                Translation2d tagTranslation = new Translation2d(0.45, -0.14).plus(offset);
                tagTranslation.rotateBy(tagRotation);
                return tagPose.plus(new Transform2d(tagTranslation, Rotation2d.k180deg));
            }
        }
    }

    private static final String LOGGING_PREFIX = "Path On The Fly Poses: ";

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

    public static Command pathFindToProcessor(CommandSwerveDrivetrain drivetrain){
        final int processorTagIDBlue = 16;
        final int processorTagIDRed = 3;

        final Pose2d processorPoseBlue = Constants.FIELD_LAYOUT.getTagPose(processorTagIDBlue).get().toPose2d().plus(new Transform2d(0.5, 0.0,Rotation2d.kCCW_90deg));
        final Pose2d processorPoseRed = Constants.FIELD_LAYOUT.getTagPose(processorTagIDRed).get().toPose2d().plus(new Transform2d(0.5, 0.0,Rotation2d.kCCW_90deg));

        Logger.recordOutput(LOGGING_PREFIX + "Processor: Blue", processorPoseBlue);
        Logger.recordOutput(LOGGING_PREFIX + "Processor: Red", processorPoseRed);

        return Commands.either(
                pathFindToPose(drivetrain, processorPoseRed),
                pathFindToPose(drivetrain, processorPoseBlue),
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
    public static Command pathFindToPose(CommandSwerveDrivetrain drivetrain, Pose2d endPose) {
        // test constraint
        PathConstraints constraints = new PathConstraints(3.0, 3.0, 2 * Math.PI, 4 * Math.PI);
        return CommandLogger.logCommand(AutoBuilder.pathfindToPose(endPose, constraints), "PathFind")
            .andThen(DriveToPose.getCommand(drivetrain, endPose));
    }

    /**
     * Moves to the closest reef scoring position on the robot's alliance side
     *
     * @param drivetrain
     * @param currentBotPose supplier for the robot's current position
     * @param right          if true, move to the right pole of the reef. If false,
     *                       move to the left pole
     * @return Command to path find to the reef
     */
    public static Command pathfindToReef(CommandSwerveDrivetrain drivetrain, Supplier<Pose2d> currentBotPose, boolean right) {
        // Create commands
        return Commands.either(
                pathfindToReefAllianceSpecific(drivetrain, currentBotPose, right, true),
                pathfindToReefAllianceSpecific(drivetrain, currentBotPose, right, false),
                () -> DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red
        );
    }

    private static Command pathfindToReefAllianceSpecific(CommandSwerveDrivetrain drivetrain, Supplier<Pose2d> currentBotPose, boolean right, boolean isRed) {
        Map<Integer, Pose2d> tagIDPositionMapRight = logPositions(ReefPositions.RIGHT_TAG_ID_TO_POSITION_DYNAMIC, "RIGHT_TAG_ID_TO_POSITION_DYNAMIC");
        Map<Integer, Pose2d> tagIDPositionMapLeft = logPositions(ReefPositions.LEFT_TAG_ID_TO_POSITION_DYNAMIC, "LEFT_TAG_ID_TO_POSITION_DYNAMIC");
        return Commands.either(
                pathfindToReefHelper(drivetrain, currentBotPose, tagIDPositionMapRight, isRed),
                pathfindToReefHelper(drivetrain, currentBotPose, tagIDPositionMapLeft, isRed),
                () -> right);
    }

    private static Map<Integer, Pose2d> logPositions(Map<Integer, Pose2d> tagIDPositionMap, String name) {
        for (int key : tagIDPositionMap.keySet()) {
            Logger.recordOutput(LOGGING_PREFIX + "Reef " + name + ": " + key, tagIDPositionMap.get(key));
        }
        return tagIDPositionMap;
    }
    /**
     * Helper method to fill out the select commands for the left and right sides of
     * the reef
     *
     * @param drivetrain
     * @param currentBotPose
     * @param tagIDPositionMap
     * @return
     */
    private static Command pathfindToReefHelper(CommandSwerveDrivetrain drivetrain, Supplier<Pose2d> currentBotPose,
            Map<Integer, Pose2d> tagIDPositionMap, boolean isRed) {
        return Commands.select(buildPathfindToReefCommandMap(drivetrain, tagIDPositionMap),
                () -> getNearestReefTagID(currentBotPose.get(), isRed));
    }

    private static Map<Integer, Command> buildPathfindToReefCommandMap(CommandSwerveDrivetrain drivetrain, Map<Integer, Pose2d> tagIDPositionMap) {
        return ReefPositions.REEF_TAG_IDS.stream()
                .map(tagId -> buildPathfindToReefCommandEntry(drivetrain, tagId, tagIDPositionMap))
                .collect(Collectors.toMap(
                        Map.Entry::getKey,
                        Map.Entry::getValue));
    }

    private static Entry<Integer, Command> buildPathfindToReefCommandEntry(CommandSwerveDrivetrain drivetrain, int tagID,
            Map<Integer, Pose2d> tagIDPositionMap) {
        Pose2d endPose = tagIDPositionMap.get(tagID);
        return Map.entry(tagID, pathFindToPose(drivetrain, endPose));
    }

    private static int getNearestReefTagID(Pose2d currentBotPose, boolean isRed) {
        List<AprilTag> tags = ReefPositions.REEF_TAGS;
        // If red alliance
        if (isRed) {
            tags = ReefPositions.REEF_TAGS_RED;
            // If blue alliance
        } else {
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
