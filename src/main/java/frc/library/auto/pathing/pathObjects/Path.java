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
import java.util.List;
import java.util.function.Supplier;

import org.json.simple.parser.ParseException;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
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
import frc.library.auto.pathing.PurePursuitSettings;
import frc.library.auto.pathing.field.GameField;

import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.trajectory.PathPlannerTrajectory;
import com.pathplanner.lib.trajectory.PathPlannerTrajectoryState;
import com.pathplanner.lib.util.FileVersionException;

public class Path implements Iterable<PathPoint> {

    private ArrayList<PathPoint> points;
    private double lastPointTolerance;
    public PurePursuitSettings config;
    private Alliance builtAlliance;

    /**
     * Subscribes to the alliance color in FMS data.
     * The complete path is /FMSInfo/IsRedAlliance.
     */
    private BooleanSubscriber isRedAllianceSubScriber = 
        NetworkTableInstance.getDefault()
            .getTable("FMSInfo")
            .getBooleanTopic("IsRedAlliance")
            .subscribe(false);

    /**
     * Constructs a path from a path planner file name.
     * @param config Configuration for the pure pursuit library.
     * @param originAlliance The alliance this path is built for.
     * @param pathName The path planner file name.
     * @return A new path that matches the path planner path.
     */
    public static Path getFromPathPlanner(PurePursuitSettings config, Alliance originAlliance, String pathName) {
        PathPlannerPath pathPlannerPath;
        try {
            pathPlannerPath = PathPlannerPath.fromPathFile(pathName);
        } catch (FileVersionException | IOException | ParseException e) {
            e.printStackTrace();
            System.out.println("Failed to open path " + pathName);
            return null;
        }
        
        ArrayList<PathPoint> pathPoints = new ArrayList<PathPoint>();

        // pathPlannerPathPoint.rotationTarget is sometimes null, this block of code is correcting that
        // by interpolating between KNOWN rotation values, and overriding the null ones before pathPoints are constructed.
        
        com.pathplanner.lib.path.PathPoint initialPoint = pathPlannerPath.getPoint(0);
        Rotation2d initialHeading = pathPlannerPath.getInitialHeading();
        // The position of points that still need rotation patching
        
        int patchedIndex = 0;
        List<com.pathplanner.lib.path.PathPoint> pathPlannerPathPoints = pathPlannerPath.getAllPathPoints();
        for (int foundTargetIndex = 0; foundTargetIndex < pathPlannerPathPoints.size(); foundTargetIndex++) {
            // Loop until rotation is found
            if (pathPlannerPathPoints.get(foundTargetIndex).rotationTarget != null) {
                com.pathplanner.lib.path.PathPoint finalPoint = pathPlannerPathPoints.get(foundTargetIndex);
                Rotation2d finalHeading = finalPoint.rotationTarget.rotation();
                
                // Iterate from the position of points that still need patching, up to the index of the found rotation target.
                for (int pointIndex = patchedIndex; pointIndex <= foundTargetIndex; pointIndex++) {
                    com.pathplanner.lib.path.PathPoint currentPathPlannerPoint = pathPlannerPathPoints.get(pointIndex);

                    // Interpolate from initialHeader to finalHeading
                    Rotation2d rotation = initialHeading.interpolate(finalHeading, 
                        // FInd percent from initial to final point
                        (currentPathPlannerPoint.distanceAlongPath - initialPoint.distanceAlongPath) / 
                        (finalPoint.distanceAlongPath - initialPoint.distanceAlongPath)
                    );

                    pathPoints.add(
                        new PathPoint(config.field, currentPathPlannerPoint.position, rotation, currentPathPlannerPoint.maxV)
                    );
                }

                // Increment points and headings, and set patchedIndex to the next point
                initialPoint = finalPoint;
                initialHeading = finalHeading;
                patchedIndex = foundTargetIndex + 1;
            }


        }

        System.out.println(pathPoints.toString());

        return new Path(config, originAlliance, pathPoints.toArray(new PathPoint[pathPoints.size()]));
    }

    /**
     * Constructs a path from a path planner file name.
     * @param config Configuration for the pure pursuit library.
     * @param pathName The path planner file name.
     * @return A new path that matches the path planner path.
     */
    public static Path getFromPathPlanner(PurePursuitSettings config, String pathName) {
        return Path.getFromPathPlanner(config, config.originAlliance, pathName);
    }

    /**
     * Constructs a path from a path weaver file name. Paths must be inserted
     * into the src\main\deploy\paths directory
     * @param config Configuration for the pure pursuit library.
     * @param originAlliance The alliance this path is built for.
     * @param pathName The path weaver file name.
     * @return A new path that matches the path planner path.
     */
    public static Path getFromPathWeaver(PurePursuitSettings config, Alliance originAlliance, String pathName) {
        Trajectory trajectory = new Trajectory();
        try {
            trajectory = TrajectoryUtil.fromPathweaverJson(
                Filesystem.getDeployDirectory().toPath().resolve("paths/" + pathName)
            );
        } catch (IOException e) {
            throw new UncheckedIOException(e);
        }
        return new Path(config, originAlliance, trajectory);
    }

    /**
     * Constructs a path from a path weaver file name. Paths must be inserted
     * into the src\main\deploy\paths directory
     * @param config Configuration for the pure pursuit library.
     * @param pathName The path weaver file name.
     * @return A new path that matches the path planner path.
     */
    public static Path getFromPathWeaver(PurePursuitSettings config, String pathName) {
        return getFromPathWeaver(config, config.originAlliance, pathName);
    }

    /**
     * Constructs a path from a given set of points,
     * 0.02 meters is set as the default end point tolerance.
     * @param config Configuration for the pure pursuit library.
     * @param originAlliance The alliance this path is built for.
     * @param Points All points along the path, the first point passed 
     * into this initializer is the first point along the path.
     */
    public Path(PurePursuitSettings config, Alliance originAlliance, PathPoint... Points) {
        this(config, originAlliance, config.endpointTolerance, new ArrayList<PathPoint>(Arrays.asList(Points)));
    }

    /**
     * Constructs a path from a given set of points,
     * 0.02 meters is set as the default end point tolerance.
     * @param config Configuration for the pure pursuit library.
     * @param Points All points along the path, the first point passed 
     * into this initializer is the first point along the path.
     */
    public Path(PurePursuitSettings config, PathPoint... Points) {
        this(config, config.originAlliance, config.endpointTolerance, new ArrayList<PathPoint>(Arrays.asList(Points)));
    }

    /**
     * Constructs a path from a given set of points.
     * @param config Configuration for the pure pursuit library.
     * @param originAlliance The alliance this path is built for.
     * @param lastPointTolerance The distance to the last point where the path ends.
     * @param Points All points along the path, the first point passed 
     * into this initializer is the first point along the path.
     */
    public Path(PurePursuitSettings config, Alliance originAlliance, double lastPointTolerance, PathPoint... Points) {
        this(config, originAlliance, lastPointTolerance, new ArrayList<PathPoint>(Arrays.asList(Points)));
    }

    /**
     * Constructs a path from a given set of points.
     * @param config Configuration for the pure pursuit library.
     * @param lastPointTolerance The distance to the last point where the path ends.
     * @param Points All points along the path, the first point passed 
     * into this initializer is the first point along the path.
     */
    public Path(PurePursuitSettings config, double lastPointTolerance, PathPoint... Points) {
        this(config, config.originAlliance, lastPointTolerance, new ArrayList<PathPoint>(Arrays.asList(Points)));
    }

    /**
     * Constructs a path given a path weaver trajectory.
     * @param config Configuration for the pure pursuit library.
     * @param originAlliance The alliance this path is built for.
     * @param lastPointTolerance The distance to the last point where the path ends.
     * @param pathWeaverTrajectory The PathWeaver trajectory.
     */
    public Path(PurePursuitSettings config, Alliance originAlliance, double lastPointTolerance, Trajectory pathWeaverTrajectory) {
        ArrayList<PathPoint> tempPoints = new ArrayList<PathPoint>();
        for (State state : pathWeaverTrajectory.getStates()) {
            tempPoints.add(new PathPoint(config.field, state.poseMeters, state.velocityMetersPerSecond));
        }
        processPoints(config, originAlliance, lastPointTolerance, tempPoints);
    }

    /**
     * Constructs a path given a path planner trajectory.
     * @param config Configuration for the pure pursuit library.
     * @param originAlliance The alliance this path is built for.
     * @param lastPointTolerance The distance to the last point where the path ends.
     * @param pathPlannerTrajectory The PathPlanner trajectory.
     */
    @Deprecated
    public Path(PurePursuitSettings config, Alliance originAlliance, double lastPointTolerance, PathPlannerTrajectory pathPlannerTrajectory) {
        ArrayList<PathPoint> tempPoints = new ArrayList<PathPoint>();
        for (PathPlannerTrajectoryState state : pathPlannerTrajectory.getStates()) {
            tempPoints.add(new PathPoint(config.field, state.pose, state.linearVelocity));
        }
        processPoints(config, originAlliance, lastPointTolerance, tempPoints);
    }

    /**
     * Constructs a path given a path weaver trajectory.
     * @param config Configuration for the pure pursuit library.
     * @param originAlliance The alliance this path is built for.
     * @param pathWeaverTrajectory The PathWeaver trajectory.
     */
    public Path(PurePursuitSettings config, Alliance originAlliance, Trajectory pathWeaverTrajectory) {
        this(config, originAlliance, config.endpointTolerance, pathWeaverTrajectory);
    }

    /**
     * Constructs a path given a path weaver trajectory.
     * @param config Configuration for the pure pursuit library.
     * @param pathWeaverTrajectory The PathWeaver trajectory.
     */
    public Path(PurePursuitSettings config, Trajectory pathWeaverTrajectory) {
        this(config, config.originAlliance, config.endpointTolerance, pathWeaverTrajectory);
    }

    /**
     * Constructs a path given a path planner trajectory.
     * @param config Configuration for the pure pursuit library.
     * @param originAlliance The alliance this path is built for.
     * @param pathWeaverTrajectory The PathPlanner trajectory.
     */
    public Path(PurePursuitSettings config, Alliance originAlliance, PathPlannerTrajectory pathPlannerTrajectory) {
        this(config, originAlliance, config.endpointTolerance, pathPlannerTrajectory);
    }

    /**
     * Constructs a path given a path planner trajectory.
     * @param config Configuration for the pure pursuit library.
     * @param originAlliance The alliance this path is built for.
     * @param pathWeaverTrajectory The PathPlanner trajectory.
     */
    public Path(PurePursuitSettings config, PathPlannerTrajectory pathPlannerTrajectory) {
        this(config, config.originAlliance, config.endpointTolerance, pathPlannerTrajectory);
    }

    /**
     * Constructs a path from multiple points
     * @param Points ArrayList<PathPoint> the first point passed
     * into this initializer is the first point along the path.
     * @param lastPointTolerance double in meters
     */
    public Path(PurePursuitSettings config, Alliance originAlliance, double lastPointTolerance, ArrayList<PathPoint> Points) {
        processPoints(config, originAlliance, lastPointTolerance, Points);
    }

    /**
     * Processes points to ensure the path is physically drivable.
     * @param config Configuration for the pure pursuit library.
     * @param originAlliance The alliance this path is built for.
     * @param lastPointTolerance The distance to the last point where the path ends.
     * @param points All points along the path, the first point passed 
     * into this initializer is the first point along the path.
     */
    void processPoints(PurePursuitSettings config, Alliance originAlliance, double lastPointTolerance, ArrayList<PathPoint> points) {
        System.out.println();

        System.out.println("Processing path " + this.toString());
        this.config = config;
        this.lastPointTolerance = lastPointTolerance;
        this.builtAlliance = config.originAlliance;
        this.points = points;

        if (!isValid()) {
            DriverStation.reportError("Malformed path: A path cannot be formed with less than three points.", true);
            return;
        }

        // Parse through a copy, as the original is being edited
        ArrayList<PathPoint> pointsCopy = new ArrayList<PathPoint>(points);

        // Increment rotation of each point by the forward direction angle
        for (int index = 0; index < points.size(); index++) {
            pointsCopy.set(index, points.get(index).rotateBy(config.forwardAngle));
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
            double lookAheadMeters = calculateLookAhead(previousPoint.speedMetersPerSecond);

            // Pull max deceleration from constants
            if (deceleration > config.maxAccelerationMeters) {

                // Clamp speed
                double previousSpeed = previousPoint.speedMetersPerSecond;
                // This index will remain unaffected
                points.get(i-1).speedMetersPerSecond = 
                    point.speedMetersPerSecond + deltaD * config.maxAccelerationMeters;
                System.out.println(
                    "Clamped speed from " + previousSpeed + " to " + 
                    previousPoint.speedMetersPerSecond
                );

            } else if (
                deceleration < config.maxAccelerationMeters && 
                deltaS < 0 && 
                (
                    1 + (deltaS / (config.maxAccelerationMeters * deltaD) - lookAheadMeters/deltaD) > 0
                    || i == pointsCopy.size()-1 // Ignore the above if we are checking the last point
                )
            ) {

                // Insert new point
                // Normalized, deltaS / Swerve.MaxAccelerationMeters is negative
                double percentFromLastPoint =  1 + (deltaS / (config.maxAccelerationMeters * deltaD));
                if (i != pointsCopy.size()-1) {
                    percentFromLastPoint -= lookAheadMeters / deltaD;
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

    public double calculateLookAhead(double speed) {
        return MathUtil.clamp(speed * config.lookAheadScalar, 0.1, 2);
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
        
        // Flip the built alliance
        if (builtAlliance.equals(Alliance.Red)) {
            builtAlliance = Alliance.Blue;
        } else {
            builtAlliance = Alliance.Red;
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
     * @return A supplier resolved with the first point on the path
     */
    public Supplier<Pose2d> getStartingPoseSupplier() {
        return new Supplier<Pose2d>() {
            @Override
            public Pose2d get() {
                return first();
            }
        };
    }

    /**
     * @return The first point on the path.
     */
    public PathPoint first() {
        return get(0);
    }

    /**
     * @return The last point on the path.
     */
    public PathPoint last() {
        return get(size() - 1);
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
        return config.field;
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
