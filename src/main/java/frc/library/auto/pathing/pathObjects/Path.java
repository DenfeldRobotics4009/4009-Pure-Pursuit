// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.library.auto.pathing.pathObjects;

import java.io.IOException;
import java.io.UncheckedIOException;
import java.util.ArrayList;
import java.util.Arrays;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.Trajectory.State;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import frc.library.auto.pathing.PurePursuitController;
import frc.library.auto.pathing.PurePursuitSettings;

import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPlannerTrajectory;

public class Path {

    public ArrayList<PathPoint> points;

    public double lastPointTolerance;

    /**
     * Constructs a path from a path planner file name.
     */
    public static Path getFromPathPlanner(String pathName) {
        PathPlannerPath pathPlannerPath = PathPlannerPath.fromPathFile(pathName);   
        PathPlannerTrajectory trajectory = pathPlannerPath.getTrajectory(
            new ChassisSpeeds(), pathPlannerPath.getPreviewStartingHolonomicPose().getRotation());
        return new Path(trajectory);
    }

    /**
     * Constructs a path from a path weaver file name. Paths must be inserted
     * into the src\main\deploy\paths directory
     */
    public static Path getFromPathWeaver(String pathName) {
            Trajectory trajectory = new Trajectory();
            try {
                trajectory = TrajectoryUtil.fromPathweaverJson(
                    Filesystem.getDeployDirectory().toPath().resolve("paths/" + pathName)
                );
            } catch (IOException e) {
                throw new UncheckedIOException(e);
            }
            return new Path(trajectory);
    }
    
    /**
     * Constructs a path from a given set of points,
     * 0.02 meters is set as the default end point tolerance
     * @param Points the first point passed into this 
     * initializer is the first point along the path.
     */
    public Path(PathPoint... Points) {
        this(PurePursuitSettings.endpointTolerance, new ArrayList<PathPoint>(Arrays.asList(Points)));
    }

    /**
     * Constructs a path from a given set of points,
     * 0.2 meters is set as the default end point tolerance
     * @param lastPointTolerance meters, the path will finish
     * when the robot is within this distance of the last point.
     * @param Points the first point passed into this 
     * initializer is the first point along the path.
     */
    public Path(double lastPointTolerance, PathPoint... Points) {
        this(lastPointTolerance, new ArrayList<PathPoint>(Arrays.asList(Points)));
    }

    /**
     * Constructs a path given a pure pursuit trajectory.
     * @param lastPointTolerance meters, the path will finish
     * when the robot is within this distance of the last point.
     * @param pathWeaverTrajectory
     */
    public Path(double lastPointTolerance, Trajectory pathWeaverTrajectory) {
        ArrayList<PathPoint> tempPoints = new ArrayList<PathPoint>();
        for (State state : pathWeaverTrajectory.getStates()) {
            tempPoints.add(new PathPoint(state.poseMeters, state.velocityMetersPerSecond));
        }
        process(lastPointTolerance, tempPoints);
    }

    /**
     * Constructs a path given a pure pursuit trajectory.
     * @param lastPointTolerance meters, the path will finish
     * when the robot is within this distance of the last point.
     * @param pathPlannerTrajectory
     */
    public Path(double lastPointTolerance, PathPlannerTrajectory pathPlannerTrajectory) {
        ArrayList<PathPoint> tempPoints = new ArrayList<PathPoint>();
        for (com.pathplanner.lib.path.PathPlannerTrajectory.State state : pathPlannerTrajectory.getStates()) {
            tempPoints.add(new PathPoint(state.positionMeters, state.targetHolonomicRotation, state.velocityMps));
        }
        process(lastPointTolerance, tempPoints);
    }

    /**
     * Constructs a path given a pure pursuit trajectory,
     * 0.2 meters is set as the default end point tolerance.
     * @param pathPlannerTrajectory
     */
    public Path(Trajectory pathWeaverTrajectory) {
        this(PurePursuitSettings.endpointTolerance, pathWeaverTrajectory);
    }

    /**
     * Constructs a path given a pure pursuit trajectory,
     * 0.2 meters is set as the default end point tolerance.
     * @param pathPlannerTrajectory
     */
    public Path(PathPlannerTrajectory pathPlannerTrajectory) {
        this(PurePursuitSettings.endpointTolerance, pathPlannerTrajectory);
    }

    /**
     * Constructs a path from multiple points
     * @param Points ArrayList<PathPoint> the first point passed
     * into this initializer is the first point along the path.
     * @param lastPointTolerance double in meters
     */
    public Path(double lastPointTolerance, ArrayList<PathPoint> Points) {
        process(lastPointTolerance, Points);
    }

    void process(double lastPointTolerance, ArrayList<PathPoint> Points) {
        System.out.println();

        System.out.println("Processing path " + this.toString());

        // // Flip all points to the corresponding side
        // if (DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red) {
        //     System.out.println("Flipping point coordinates to red alliance");
        //     for (PathPoint pathPoint : Points) {
        //         Pose2d flippedPoint = Field.mirrorPoint(new Pose2d(pathPoint.posMeters, pathPoint.orientation));
        //         pathPoint.posMeters = flippedPoint.getTranslation();
        //         pathPoint.orientation = flippedPoint.getRotation();
        //     }
        // } else {
        //     System.out.println("Assuming blue alliance");
        // }

        this.lastPointTolerance = lastPointTolerance;
        points = Points;

        if (!isValid()) {
            DriverStation.reportError("Malformed path: A path cannot be formed with less than three points.", true);
            return;
        }

        // Increment rotation of each point by the forward direction angle
        for (PathPoint pathPoint : Points) {
            pathPoint = new PathPoint(
                pathPoint.getTranslation(), 
                pathPoint.getRotation().plus(PurePursuitSettings.forwardAngle), 
                pathPoint.speedMetersPerSecond
            );
        }

        // Parse through a copy, as the original is being edited
        ArrayList<PathPoint> pointsCopy = new ArrayList<PathPoint>(Points);
        // Parse backward to correct speed of points
        // Parse from back, end at the first
        for (int i = pointsCopy.size()-1; i > 0; i--) {
            
            PathPoint point = pointsCopy.get(i);
            PathPoint previousPoint = pointsCopy.get(i-1);
            double deltaS = point.speedMetersPerSecond - previousPoint.speedMetersPerSecond;
            double deltaD = point.getDistance(previousPoint);
            // in this case, acceleration is negative, deceleration is positive
            double deceleration = -(deltaS / deltaD);

            // Calculate lookahead for deceleration point offset
            double lookAheadMeters = PurePursuitController.calculateLookAhead(
                previousPoint.speedMetersPerSecond
            );

            // Pull max deceleration from constants
            if (deceleration > PurePursuitSettings.maxAccelerationMeters) {

                // Clamp speed
                double previousSpeed = previousPoint.speedMetersPerSecond;
                // This index will remain unaffected
                Points.get(i-1).speedMetersPerSecond = 
                    point.speedMetersPerSecond + deltaD*PurePursuitSettings.maxAccelerationMeters;
                System.out.println(
                    "Clamped speed from " + previousSpeed + " to " + 
                    previousPoint.speedMetersPerSecond
                );

            } else if (
                deceleration < PurePursuitSettings.maxAccelerationMeters && 
                deltaS < 0 && 
                (
                    1 + (deltaS / (PurePursuitSettings.maxAccelerationMeters * deltaD) - lookAheadMeters/deltaD) > 0
                    || i == pointsCopy.size()-1 // Ignore the above if we are checking the last point
                )
            ) {

                // Insert new point
                // Normalized, deltaS / Swerve.MaxAccelerationMeters is negative
                double percentFromLastPoint =  1 + (deltaS / (PurePursuitSettings.maxAccelerationMeters * deltaD));
                if (i != pointsCopy.size()-1) {
                    percentFromLastPoint -= lookAheadMeters/deltaD;
                }
                System.out.println(
                    "Inserted new point at index " + i + " at " + percentFromLastPoint*100 + "%");
                // Interpolate between, and set speed to last speed
                PathPoint insertedPoint = previousPoint.interpolate(
                    point, percentFromLastPoint
                );
                insertedPoint.speedMetersPerSecond = previousPoint.speedMetersPerSecond;
                points.add(i, insertedPoint);
            }
        }

        // Parse through point and print data
        for (int i = 0; i < points.size(); i++) {
            System.out.print("Point " + i);
            System.out.print(" - " + new Pose2d(points.get(i).getTranslation(), points.get(i).getRotation()));
            System.out.println(" - " + points.get(i).speedMetersPerSecond + " meters/sec");
        }

        System.out.println();
    }

    /**
     * Checks if the path is valid
     * @return true if the path is valid
     */
    public boolean isValid() {
        if (points.size() < 3) {
            return false;
        }

        return true;
    }

    public ArrayList<Pose2d> getPose2ds() {
        return new ArrayList<Pose2d>(points);
    }

    public void setLastPointTolerance(double lastPointTolerance) {
        this.lastPointTolerance = MathUtil.clamp(lastPointTolerance, 0, 10);
    }

    public Pose2d getStartingPose() {
        return getPose2ds().get(0);
    }
} 
