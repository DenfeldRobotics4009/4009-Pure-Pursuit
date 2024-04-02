// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.library.auto.pathing.pathObjects;

import java.util.ArrayList;
import java.util.Arrays;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.Trajectory.State;
import edu.wpi.first.wpilibj.DriverStation;
import frc.library.auto.pathing.PurePursuitController;

public class Path {

    public ArrayList<PathPoint> points;

    public double lastPointTolerance;
    
    /**
     * Constructs a path from a given set of points,
     * 0.02 meters is set as the default end point tolerance
     * @param Points the first point passed into this 
     * initializer is the first point along the path.
     */
    public Path(PathPoint... Points) {
        this(PurePursuitController.endpointTolerance, new ArrayList<PathPoint>(Arrays.asList(Points)));
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
     * Constructs a path given a pure pursuit trajectory,
     * The trajectory is sampled every half second along its
     * route to form each pathPoint.
     * @param lastPointTolerance meters, the path will finish
     * when the robot is within this distance of the last point.
     * @param pathWeaverTrajectory
     * @param samplesPerSecond the rate to add points to the path from
     * the trajectory, higher samples per second means more points, and 
     * a more accurate path. High sample rates may lead to high memory usage.
     */
    public Path(double lastPointTolerance, Trajectory pathWeaverTrajectory, double samplesPerSecond) {
        ArrayList<PathPoint> tempPoints = new ArrayList<PathPoint>();
        for (int i = 0; i < pathWeaverTrajectory.getTotalTimeSeconds() * samplesPerSecond; i ++) {
            State currentState = pathWeaverTrajectory.sample((double) i / samplesPerSecond);

            tempPoints.add(new PathPoint(currentState.poseMeters, currentState.velocityMetersPerSecond));
        }
        // Always sample the first and last time
        State state = pathWeaverTrajectory.sample(pathWeaverTrajectory.getTotalTimeSeconds());
        tempPoints.add(new PathPoint(state.poseMeters, state.velocityMetersPerSecond));

        process(lastPointTolerance, tempPoints);
    }

    /**
     * Constructs a path given a pure pursuit trajectory,
     * 0.2 meters is set as the default end point tolerance.
     * The trajectory is sampled every half second along its
     * route to form each pathPoint.
     * @param pathWeaverTrajectory
     * @param samplesPerSecond the rate to add points to the path from
     * the trajectory, higher samples per second means more points, and 
     * a more accurate path. High sample rates may lead to high memory usage.
     */
    public Path(Trajectory pathWeaverTrajectory, double samplesPerSecond) {
        this(PurePursuitController.endpointTolerance, pathWeaverTrajectory, samplesPerSecond);
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
                pathPoint.getRotation().plus(PurePursuitController.forwardAngle), 
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
            if (deceleration > PurePursuitController.maxAccelerationMeters) {

                // Clamp speed
                double previousSpeed = previousPoint.speedMetersPerSecond;
                // This index will remain unaffected
                Points.get(i-1).speedMetersPerSecond = 
                    point.speedMetersPerSecond + deltaD*PurePursuitController.maxAccelerationMeters;
                System.out.println(
                    "Clamped speed from " + previousSpeed + " to " + 
                    previousPoint.speedMetersPerSecond
                );

            } else if (
                deceleration < PurePursuitController.maxAccelerationMeters && 
                deltaS < 0 && 
                (
                    1 + (deltaS / (PurePursuitController.maxAccelerationMeters * deltaD) - lookAheadMeters/deltaD) > 0
                    || i == pointsCopy.size()-1 // Ignore the above if we are checking the last point
                )
            ) {

                // Insert new point
                // Normalized, deltaS / Swerve.MaxAccelerationMeters is negative
                double percentFromLastPoint =  1 + (deltaS / (PurePursuitController.maxAccelerationMeters * deltaD));
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
} 
