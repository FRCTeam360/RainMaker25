// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.ArrayList;
import java.util.List;

import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

/** Add your docs here. */
public class OnTheFlyPaths {

    private enum OnFlyPath{
        TEST_PATH,
        TEST_PATH2
    }

    public static PathPlannerPath makeOnTheFlyPaths(OnFlyPath currentPathTemporary, Pose2d robotStartingPose){
        List<Waypoint> wayPoints = new ArrayList<Waypoint>();
        switch(currentPathTemporary){
            case TEST_PATH:
                    wayPoints = PathPlannerPath.waypointsFromPoses(
                    robotStartingPose,
                    new Pose2d(3.0, 1.0, Rotation2d.fromDegrees(0)),
                    new Pose2d(5.094, 5.302, Rotation2d.fromDegrees(90))
                );
                break;
            case TEST_PATH2:
                wayPoints = PathPlannerPath.waypointsFromPoses(
                robotStartingPose,
                new Pose2d(3.0, 1.0, Rotation2d.fromDegrees(0)),
                new Pose2d(5.0, 3.0, Rotation2d.fromDegrees(90))
                );
                break;
                
        }
    
        PathConstraints constraints = new PathConstraints(3.0, 3.0, 2 * Math.PI, 4 * Math.PI); // test constraints(to be changed)
    
        PathPlannerPath path = new PathPlannerPath(
                wayPoints,
                constraints,
                null,
                new GoalEndState(0.0, Rotation2d.fromDegrees(-90)));

        path.preventFlipping = true;

        return path;
    }

}
