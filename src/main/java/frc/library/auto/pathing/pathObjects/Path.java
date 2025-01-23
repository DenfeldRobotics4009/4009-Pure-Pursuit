// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.library.auto.pathing.pathObjects;

import java.io.IOException;
import java.io.UncheckedIOException;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.EnumSet;
import java.util.Iterator;
import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.Trajectory.State;
import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.networktables.NetworkTableEvent;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.library.auto.pathing.PurePursuitController;
import frc.library.auto.pathing.PurePursuitSettings;
import frc.library.auto.pathing.field.GameField;

import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPlannerTrajectory;

public class Path implements Iterable<PathPoint> {

    private ArrayList<PathPoint> points;
    private double lastPointTolerance;
    private GameField field;
    private Alliance builtAlliance;

    /**
     * Subscribes to the alliance color in FMS data.
     * The complete path is /FMSInfo/IsRedAlliance
     */
    private BooleanSubscriber isRedAllianceSubScriber = 
        NetworkTableInstance.getDefault()
            .getTable("FMSInfo")
            .getBooleanTopic("IsRedAlliance")
            .subscribe(false);

    /**
     * Constructs a path from a path planner file name.
     */
    public static Path getFromPathPlanner(GameField field, Alliance originAlliance, String pathName) {
        PathPlannerPath pathPlannerPath = PathPlannerPath.fromPathFile(pathName);   
        PathPlannerTrajectory trajectory = pathPlannerPath.getTrajectory(
            new ChassisSpeeds(), pathPlannerPath.getPreviewStartingHolonomicPose().getRotation());
        return new Path(field, originAlliance, trajectory);
    }

    /**
     * Constructs a path from a path weaver file name. Paths must be inserted
     * into the src\main\deploy\paths directory
     */
    public static Path getFromPathWeaver(GameField field, Alliance originAlliance, String pathName) {
        Trajectory trajectory = new Trajectory();
        try {
            trajectory = TrajectoryUtil.fromPathweaverJson(
                Filesystem.getDeployDirectory().toPath().resolve("paths/" + pathName)
            );
        } catch (IOException e) {
            throw new UncheckedIOException(e);
        }
        return new Path(field, originAlliance, trajectory);
    }
    
    /**
     * Constructs a path from a given set of points,
     * 0.02 meters is set as the default end point tolerance
     * @param Points the first point passed into this 
     * initializer is the first point along the path.
     */
    public Path(GameField field, Alliance originAlliance, PathPoint... Points) {
        this(field, originAlliance, PurePursuitSettings.endpointTolerance, new ArrayList<PathPoint>(Arrays.asList(Points)));
    }

    /**
     * Constructs a path from a given set of points,
     * 0.2 meters is set as the default end point tolerance
     * @param lastPointTolerance meters, the path will finish
     * when the robot is within this distance of the last point.
     * @param Points the first point passed into this 
     * initializer is the first point along the path.
     */
    public Path(GameField field, Alliance originAlliance, double lastPointTolerance, PathPoint... Points) {
        this(field, originAlliance, lastPointTolerance, new ArrayList<PathPoint>(Arrays.asList(Points)));
    }

    /**
     * Constructs a path given a pure pursuit trajectory.
     * @param lastPointTolerance meters, the path will finish
     * when the robot is within this distance of the last point.
     * @param pathWeaverTrajectory
     */
    public Path(GameField field, Alliance originAlliance, double lastPointTolerance, Trajectory pathWeaverTrajectory) {
        ArrayList<PathPoint> tempPoints = new ArrayList<PathPoint>();
        for (State state : pathWeaverTrajectory.getStates()) {
            tempPoints.add(new PathPoint(field, state.poseMeters, state.velocityMetersPerSecond));
        }
        processPoints(field, originAlliance, lastPointTolerance, tempPoints);
    }

    /**
     * Constructs a path given a pure pursuit trajectory.
     * @param lastPointTolerance meters, the path will finish
     * when the robot is within this distance of the last point.
     * @param pathPlannerTrajectory
     */
    public Path(GameField field, Alliance originAlliance, double lastPointTolerance, PathPlannerTrajectory pathPlannerTrajectory) {
        ArrayList<PathPoint> tempPoints = new ArrayList<PathPoint>();
        for (com.pathplanner.lib.path.PathPlannerTrajectory.State state : pathPlannerTrajectory.getStates()) {
            tempPoints.add(new PathPoint(field, state.positionMeters, state.targetHolonomicRotation, state.velocityMps));
        }
        processPoints(field, originAlliance, lastPointTolerance, tempPoints);
    }

    /**
     * Constructs a path given a pure pursuit trajectory,
     * 0.2 meters is set as the default end point tolerance.
     * @param pathPlannerTrajectory
     */
    public Path(GameField field, Alliance originAlliance, Trajectory pathWeaverTrajectory) {
        this(field, originAlliance, PurePursuitSettings.endpointTolerance, pathWeaverTrajectory);
    }

    /**
     * Constructs a path given a pure pursuit trajectory,
     * 0.2 meters is set as the default end point tolerance.
     * @param pathPlannerTrajectory
     */
    public Path(GameField field, Alliance originAlliance, PathPlannerTrajectory pathPlannerTrajectory) {
        this(field, originAlliance, PurePursuitSettings.endpointTolerance, pathPlannerTrajectory);
    }

    /**
     * Constructs a path from multiple points
     * @param Points ArrayList<PathPoint> the first point passed
     * into this initializer is the first point along the path.
     * @param lastPointTolerance double in meters
     */
    public Path(GameField field, Alliance originAlliance, double lastPointTolerance, ArrayList<PathPoint> Points) {
        processPoints(field, originAlliance, lastPointTolerance, Points);
    }

    /**
     * 
     * @param field
     * @param originAlliance
     * @param lastPointTolerance
     * @param Points
     */
    void processPoints(GameField field, Alliance originAlliance, double lastPointTolerance, ArrayList<PathPoint> points) {
        System.out.println();

        System.out.println("Processing path " + this.toString());
        this.field = field;
        this.lastPointTolerance = lastPointTolerance;
        this.builtAlliance = originAlliance;
        this.points = points;

        if (!isValid()) {
            DriverStation.reportError("Malformed path: A path cannot be formed with less than three points.", true);
            return;
        }

        // Parse through a copy, as the original is being edited
        ArrayList<PathPoint> pointsCopy = new ArrayList<PathPoint>(points);

        // Increment rotation of each point by the forward direction angle
        for (int index = 0; index < points.size(); index++) {
            pointsCopy.set(index, points.get(index).rotateBy(PurePursuitSettings.forwardAngle));
        }

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
                points.get(i-1).speedMetersPerSecond = 
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
        // for (int i = 0; i < points.size(); i++) {
        //     System.out.print("Point " + i);
        //     System.out.print(" - " + new Pose2d(points.get(i).getTranslation(), points.get(i).getRotation()));
        //     System.out.println(" - " + points.get(i).speedMetersPerSecond + " meters/sec");
        // }

        System.out.println();

        // Start listening to changes in alliance color
        listenToAlliance();
    }

    /**
     * Start listening to changes in the alliance color, this function should be called once path
     * processing should be finished.
     * 
     * Immediately after this function is called, the alliance this path is build for is validated
     * and if incorrect, processing will occur again. If the /FMSInfo/IsRedAlliance network table 
     * is altered after this function has finished, processing will occur again.
     */
    void listenToAlliance() {
        // Check immediately, who knows what happened before this was called
        if (isRedAllianceSubScriber.get() != (builtAlliance.equals(Alliance.Red))) {
            flipAllianceOrigin();
        }
        
        NetworkTableInstance.getDefault().addListener(
            isRedAllianceSubScriber,
            EnumSet.of(NetworkTableEvent.Kind.kValueAll),
            event -> {
                // Called when the FMS alliance changes, if this ever happens,
                // rebuild this path via the process function.
                // Is red alliance
                System.out.println("Switched alliance color to " + event.valueData.value.getBoolean());
                flipAllianceOrigin();
            }
        );
    }

    /**
     * Mutator, Flips all points to the corresponding coordinate position for the opposite alliance.
     * @return This path.
     */
    Path flipAllianceOrigin() {
        ArrayList<PathPoint> flippedPoints = new ArrayList<PathPoint>();
        for (PathPoint point : this) {
            flippedPoints.add(point.flipAllianceOrigin());
        }
        this.points = flippedPoints;
        return this;
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

    /**
     * @return ArrayList<Pose2d> of all points in the path.
     */
    public ArrayList<Pose2d> getPose2ds() {
        return new ArrayList<Pose2d>(points);
    }

    /**
     * @return ArrayList<PathPoint> of all points in the path.
     */
    public ArrayList<PathPoint> getPoints() {
        return points;
    }

    /**
     * Sets the distance in meters from the last point before the path ends.
     * @param lastPointTolerance The distance in meters from the last point before
     * the path ends.
     */
    public void setLastPointTolerance(double lastPointTolerance) {
        this.lastPointTolerance = MathUtil.clamp(lastPointTolerance, 0, 10);
    }

    /**
     * @return Distance in meters from the last point before the path ends.
     */
    public double getLastPointTolerance() {
        return this.lastPointTolerance;
    }

    /**
     * @return PathPoint of the first point on the path.
     */
    public PathPoint getStartingPose() {
        return get(0);
    }

    /**
     * @return A supplier resolved with the first point on the path
     */
    public Supplier<Pose2d> getStartingPoseSupplier() {
        return new Supplier<Pose2d>() {
            @Override
            public Pose2d get() {
                return getStartingPose();
            }
        };
    }

    /**
     * @return The alliance this path was constructed for.
     */
    public Alliance getAlliance() {
        return builtAlliance;
    }

    /**
     * @return The field this path was constructed on.
     */
    public GameField getField() {
        return field;
    }

    /**
     * @return Number of points in the path.
     */
    public int size() {
        return points.size();
    }

    /**
     * Returns the PathPoint at index.
     * @param index The index of the PathPoint.
     * @return PathPoint at index.
     */
    public PathPoint get(int index) {
        return points.get(index);
    }

    /**
     * Creates an iterator through all points along this path.
     * @return A new Iterator<PathPoint>
     */
    @Override
    public Iterator<PathPoint> iterator() {
        return new Iterator<PathPoint>() {
            int index = 0;

            @Override
            public boolean hasNext() {
                return index < (points.size() - 1);
            }

            @Override
            public PathPoint next() {
                return points.get(index++);
            }
        };
    }
} 
