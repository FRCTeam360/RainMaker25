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

    public static PathPlannerPath makeOnTheFlyPaths(OnFlyPath currentPath, Pose2d robotStartingPose, Rotation2d robotDriveHeading, double robotSpeedMeterPerSec){
        List<Waypoint> wayPoints = new ArrayList<Waypoint>();
        switch(currentPath){
            case TEST_PATH:
                    wayPoints = PathPlannerPath.waypointsFromPoses(
                    robotStartingPose,
                    new Pose2d(5.094, 5.302, Rotation2d.fromDegrees(-33.917))
                );
                break;         
        }
    }

}
