// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.library.auto.pathing;

import java.util.ArrayList;
import java.util.Arrays;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.library.auto.pathing.controllers.RotationController;
import frc.library.auto.pathing.controllers.TranslationController;
import frc.library.auto.pathing.pathObjects.Path;
import frc.library.auto.pathing.pathObjects.PathPoint;
import frc.library.auto.pathing.pathObjects.PathState;

public class PurePursuitController extends Command implements RotationController, TranslationController {

    // Set of processed points
    public final Path path;

    // Current index along the path
    int lastCrossedPointIndex = 0;
    // Distance down the path to drive towards
    double lookAheadMeters = 0.4;// Initial at 10 cm
    Pose2d lastPosition; // Initialize as null to avoid a path ending before its set for the first time

    PIDController rotationController = new PIDController(turningP, turningI, turningD);

    // The distance the robot must be within from the last point
    public static double endpointTolerance = 0.4;
    // The front of the robot
    public static Rotation2d forwardAngle = new Rotation2d();
    // LookAhead distance = lookAheadScalar * speed
    public static double lookAheadScalar = 1.5;
    // The maximum speed the robot can travel
    public static double maxVelocityMeters = 5.06; // Mk4I Swerve Module
    // The maximum speed at which the robot can accelerate, however,
    // this metric is only used in the case of decelerating along the path.
    public static double maxAccelerationMeters = 2.7;
    public static double turningP = 5, turningI = 0, turningD = 0;
    // If the robot comes this close to its goal, it will increment 
    // the last crossed point index.
    public static double distanceToGoalTolerance = endpointTolerance * 0.5;    

    PurePursuitDiagnostics diagnostics;

    double lastDistanceToGoal = lookAheadMeters;

    public PurePursuitController(Path path) {
        this.path = path;
        rotationController.enableContinuousInput(-Math.PI, Math.PI);
    }

    /**
     * Constructs a path from a given set of points,
     * 0.02 meters is set as the default end point tolerance
     * @param Points the first point passed into this 
     * initializer is the first point along the path.
     */
    public PurePursuitController(PathPoint... Points) {
        this(new Path(Points));
    }

    /**
     * Constructs a path from a given set of points,
     * 0.2 meters is set as the default end point tolerance
     * @param lastPointTolerance meters, the path will finish
     * when the robot is within this distance of the last point.
     * @param Points the first point passed into this 
     * initializer is the first point along the path.
     */
    public PurePursuitController(double lastPointTolerance, PathPoint... Points) {
        this(new Path(lastPointTolerance, new ArrayList<PathPoint>(Arrays.asList(Points))));
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
    public PurePursuitController(double lastPointTolerance, Trajectory pathWeaverTrajectory, double samplesPerSecond) {
        this(new Path(lastPointTolerance, pathWeaverTrajectory, samplesPerSecond));
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
    public PurePursuitController(Trajectory pathWeaverTrajectory, double samplesPerSecond) {
        this(new Path(pathWeaverTrajectory, samplesPerSecond));
    }

    /**
     * Configures the PID components of the steering
     * PID controller.
     * @param P
     * @param I
     * @param D
     */
    public static void setTurningPID(double P, double I, double D) {
        turningP = P;
        turningI = I;
        turningD = D;
    }

    /**
     * Sets the maximum allowed deceleration along paths
     * This does not limit acceleration, only deceleration.
     * @param maxAccelerationMeters double [0, 10]
     */
    public static void setMaxAccelerationMeters(double maxAccelerationMeters) {
        PurePursuitController.maxAccelerationMeters = 
            MathUtil.clamp(maxAccelerationMeters, 0, 10);
    }

    /**
     * Sets the maximum allowed speed of paths
     * @param maxVelocityMeters double [0, 10]
     */
    public static void setMaxVelocityMeters(double maxVelocityMeters) {
        PurePursuitController.maxVelocityMeters = 
            MathUtil.clamp(maxVelocityMeters, 0, 10);
    }

    /**
     * Sets the multiplier used to calculate lookahead
     * from the current speed of the robot along a path.
     * lookAheadMeters = speedMeters * lookAheadScalar
     * @param lookAheadScalar double, [0.1, 10]
     */
    public static void setLookAheadScalar(double lookAheadScalar) {
        PurePursuitController.lookAheadScalar = 
            MathUtil.clamp(lookAheadScalar, 0.1, 10);
    }

    /**
     * Sets the angle offset paths will be adjusted by,
     * allowing what is considered the front of the robot
     * to be changed.
     * @param angle Rotation2d
     */
    public static void setForwardAngle(Rotation2d forwardAngle) {
        PurePursuitController.forwardAngle = forwardAngle;
    }

    /**
     * Sets the default allowed distance from the robot to the
     * last point in the path for the path command to end.
     * @param tolerance
     */
    public static void setDefaultEndpointTolerance(double tolerance) {
        PurePursuitController.endpointTolerance = MathUtil.clamp(tolerance, 0, 1);
    }

    /**
     * Sets the default allowed distance from the robot to its
     * goal before incrementing last crossed point index
     * @param tolerance
     */
    public static void setDistanceToGoalTolerance(double tolerance) {
        PurePursuitController.distanceToGoalTolerance = MathUtil.clamp(tolerance, 0, 1);
    }


    // Called when the command is initially scheduled.
    @Override
    /**
     * Print path data
     */
    public void initialize() {
        lastCrossedPointIndex = 0;
        lookAheadMeters = 0.4;
        diagnostics = new PurePursuitDiagnostics(path);
        System.out.println("--- Following path of points: ---");
        for (PathPoint point : path.points) {System.out.println(point.getTranslation() + "," + point.getRotation());}
        System.out.println("--- --- --- -- --- -- --- --- ---");
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    /**
     * Operates robot functions from given driveTrain
     */
    public void execute() {

    }
    
    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        System.out.println("End of path reached");
    }

    // Returns true when the command should end.
    @Override
    /**
     * @return true when the robot is within the last point tolerance
     * and has passed the second to last point.
     */
    public boolean isFinished() {

        if (!path.isValid()) {
            return true;
        }

        // calculate distance to last point
        double distanceToLastPointMeters = getLastPoint().getDistance(lastPosition);

        return (
            // If we have passed the second to last point
            lastCrossedPointIndex >= (path.points.size() - 2) && 
            distanceToLastPointMeters < path.lastPointTolerance
        );
    }

    private static double clampStateSpeed(double stateSpeed) {
        return MathUtil.clamp(stateSpeed, 0.1, maxVelocityMeters);
    }

    public static double calculateLookAhead(double speed) {
        double clampedSpeed = clampStateSpeed(speed);
        return MathUtil.clamp(clampedSpeed * lookAheadScalar, 0.1, 2);
    }

    ChassisSpeeds getSpeeds(Pose2d robotPosition) {

        if (!path.isValid()) {
            DriverStation.reportError("Cannot follow a malformed path", false);
            return new ChassisSpeeds();
        }

        PathState state = getPathState(robotPosition);
        // The target relative to the robots current position
        lastPosition = robotPosition;
        Translation2d deltaLocation = state.goalPose.getTranslation().minus(robotPosition.getTranslation());

        // Scale to goal speed. Speed input is in meters per second, while drive accepts normal values.
        Translation2d axisSpeeds = new Translation2d(state.speedMetersPerSecond, deltaLocation.getAngle());

        // Set lookahead based upon speed of next point
        lookAheadMeters = calculateLookAhead(state.speedMetersPerSecond);

        lastDistanceToGoal = state.goalPose.getTranslation().getDistance(robotPosition.getTranslation());

        // Construct chassis speeds from state values
        // Convert field oriented to robot oriented
        return ChassisSpeeds.fromFieldRelativeSpeeds(
            // Field oriented chassisSpeeds
            new ChassisSpeeds(
                axisSpeeds.getX(),
                axisSpeeds.getY(),

                rotationController.calculate(
                    robotPosition.getRotation().getRadians(), 
                    state.goalPose.getRotation().getRadians()
                )
            ), 
            // Rotate from current direction
            robotPosition.getRotation()
        );
    }

    Translation2d calculateLookAheadGoal(double distanceAlongCurrentLine) {
        Translation2d gotoGoal;

        int pointsLookingAhead = 0;
        double distanceAlongLookaheadPoints = distanceAlongCurrentLine + lookAheadMeters;

        // Parse lookahead
        while (true) {
            // Check to make sure points are accessible
            if (lastCrossedPointIndex + pointsLookingAhead + 1 >= path.points.size()) {
                // We are looking to the end of path
                gotoGoal = path.points.get(path.points.size()-1).getTranslation();
                //println(gotoGoal);
                break;
            }
            
            // Grab 2 points, and grab the length between them
            PathPoint lookAheadPointA = path.points.get(lastCrossedPointIndex + pointsLookingAhead);
            PathPoint lookAheadPointB = path.points.get(lastCrossedPointIndex + pointsLookingAhead + 1);

            double lookAheadLineLength = lookAheadPointA.getDistance(lookAheadPointB);

            // If we are not looking past this line
            if (distanceAlongLookaheadPoints < lookAheadLineLength) {
                // Stop looping, interpolate goto
                gotoGoal = lookAheadPointA.getTranslation().interpolate(
                    lookAheadPointB.getTranslation(), 
                    distanceAlongLookaheadPoints / lookAheadLineLength // Normalized
                );

                //println(gotoGoal);
                break;
            }

            distanceAlongLookaheadPoints -= lookAheadLineLength;
            // Look 1 line ahead, and subtract length of last line
            pointsLookingAhead ++;
            // Continue
        }

        return gotoGoal;
    }

    /**
     * @param robotPosition current pose of the robot
     * @return PathState containing a goal position, rotation
     * and allowed speed to travel there.
     */
    PathState getPathState(Pose2d robotPosition) {
        Translation2d robotTranslation = robotPosition.getTranslation();

        // Store 2 points
        ArrayList<PathPoint> relevantPoints = packageRelevantPoints();

        // Assume at least 2 are grabbed
        double lengthAB = relevantPoints.get(0).getDistance(relevantPoints.get(1));
        // println("Length of current line = " + lengthAB);

        // grab perpendicular intersection
        Translation2d perpendicularIntersectionAB = PathPoint.findPerpendicularIntersection(
            relevantPoints.get(0).getTranslation(), relevantPoints.get(1).getTranslation(), robotTranslation
        );

        // Calculate position along line AB via finding difference between line length, and distance to B
        double distanceMetersAlongAB = lengthAB - relevantPoints.get(1).getTranslation().getDistance(perpendicularIntersectionAB);

        // Add the distance of the robot from the path to the distance along AB for smoothing
        //distanceMetersAlongAB += robotTranslation.getDistance(perpendicularIntersectionAB);

        // Clamp distance along AB
        distanceMetersAlongAB = MathUtil.clamp(distanceMetersAlongAB, 0, lengthAB);

        // System.out.println("Distance along AB: " + distanceMetersAlongAB);

        Translation2d gotoGoal = calculateLookAheadGoal(distanceMetersAlongAB);

        int pointsLookingAhead = 0;
        double distanceAlongLookaheadPoints = distanceMetersAlongAB + lookAheadMeters;// + lookAheadMeters*robotTranslation.getDistance(perpendicularIntersectionAB);

        // Parse lookahead
        while (true) {
            // Check to make sure points are accessible
            if (lastCrossedPointIndex + pointsLookingAhead + 1 >= path.points.size()) {
                // We are looking to the end of path
                gotoGoal = path.points.get(path.points.size()-1).getTranslation();
                //println(gotoGoal);
                break;
            }
            
            // Grab 2 points, and grab the length between them
            PathPoint lookAheadPointA = path.points.get(lastCrossedPointIndex + pointsLookingAhead);
            PathPoint lookAheadPointB = path.points.get(lastCrossedPointIndex + pointsLookingAhead + 1);

            double lookAheadLineLength = lookAheadPointA.getDistance(lookAheadPointB);

            // If we are not looking past this line
            if (distanceAlongLookaheadPoints < lookAheadLineLength) {
                // Stop looping, interpolate goto
                gotoGoal = lookAheadPointA.getTranslation().interpolate(
                    lookAheadPointB.getTranslation(), 
                    distanceAlongLookaheadPoints / lookAheadLineLength // Normalized
                );

                //println(gotoGoal);
                break;
            }

            distanceAlongLookaheadPoints -= lookAheadLineLength;
            // Look 1 line ahead, and subtract length of last line
            pointsLookingAhead ++;
            // Continue
        }

        // After finding the gotoGoal, calculate a second round of intersections perpendicular
        // to the slope formed from perpendicularIntersectionAB and gotoGoal
        

        //println("Constructing path state");
        double percentAlongAB = distanceMetersAlongAB / lengthAB;

        Rotation2d interpolatedRotation = relevantPoints.get(0).getRotation().interpolate(
            relevantPoints.get(1).getRotation(), percentAlongAB);

        double stateSpeed = 0;
        // If our speed is increasing, accelerate instantly,
        // if our speed is decreasing, limit the deceleration
        if (relevantPoints.get(0).speedMetersPerSecond < relevantPoints.get(1).speedMetersPerSecond) {
            stateSpeed = relevantPoints.get(1).speedMetersPerSecond;
        } else {
            stateSpeed = PathPoint.interpolate(
                relevantPoints.get(0).speedMetersPerSecond, 
                relevantPoints.get(1).speedMetersPerSecond, 
                percentAlongAB
            );
        }

        // Construct state
        PathState state = new PathState(gotoGoal, interpolatedRotation, clampStateSpeed(stateSpeed));

        // Check to increment index
        if (compareWithNextLine(robotTranslation)) {
            lastCrossedPointIndex ++;
            System.out.println("Increment last crossed point index to " + lastCrossedPointIndex);
        }

        if (lastDistanceToGoal < distanceToGoalTolerance) {
            lastCrossedPointIndex ++;
            DriverStation.reportWarning("Forced incrementation of last crossed point index", false);
            System.out.println("Increment last crossed point index to " + lastCrossedPointIndex);
        }

        // Update diagnostic data
        diagnostics.publishEntry(state.goalPose, robotPosition, state.speedMetersPerSecond, lastCrossedPointIndex, percentAlongAB, lookAheadMeters);

        return state;
    }

    PathPoint getLastPoint() {
        return path.points.get(path.points.size() - 1);
    }

    /**
     * Returns true if the robot is closer to the next line
     * @param positionAlongLine
     * @param robotTranslation
     * @return false if robot is closer to current line, or next line doesn't exist
     */
    boolean compareWithNextLine(Translation2d robotTranslation) {
        // Calculate the perpendicularIntersection of the next line in path, if it exists
        // Check if the line exists first
        if (lastCrossedPointIndex + 2 < path.points.size()) {
            //println("Next line found");
            // if the line exists, grab points
            PathPoint pointA = path.points.get(lastCrossedPointIndex);
            PathPoint pointB = path.points.get(lastCrossedPointIndex + 1);
            PathPoint pointC = path.points.get(lastCrossedPointIndex + 2);
            // grab perpendicular intersection
            Translation2d perpendicularIntersectionBC = PathPoint.findPerpendicularIntersection(
                pointB.getTranslation(), pointC.getTranslation(), robotTranslation
            );
            Translation2d perpendicularIntersectionAB = PathPoint.findPerpendicularIntersection(
                pointA.getTranslation(), pointB.getTranslation(), robotTranslation
            );

            //println("Found perpendicular intersection at " + perpendicularIntersectionBC);
            
            // Find distance from lines
            double distanceToAB = perpendicularIntersectionAB.getDistance(robotTranslation);
            double distanceToBC = perpendicularIntersectionBC.getDistance(robotTranslation);

            // Compare distances to each intersection
            if (distanceToAB > distanceToBC) {
                return true;
            }
        }

        return false;
    }

    /**
     * Construct and return group of 2 relevant points
     * @return Array of PathPoints with length 2
     */
    ArrayList<PathPoint> packageRelevantPoints() {
        // if our lastCrossPoint index is below zero,
        // our path doesn't contain enough points to fit the definition of a path.
        if (lastCrossedPointIndex < 0) {throw new IndexOutOfBoundsException();}

        ArrayList<PathPoint> packagedPoints = new ArrayList<PathPoint>();
        // Try to grab 2 points
        for (int i = 0; i < 2; i++) {
            try {
                packagedPoints.add(
                    path.points.get(lastCrossedPointIndex + i)
                );
            } catch (IndexOutOfBoundsException e) {
                DriverStation.reportWarning("Could not grab point at index " + (lastCrossedPointIndex + i), e.getStackTrace());
            }
        }

        // If we couldn't grab 2 points, decrement.
        if (packagedPoints.size() < 2) {
            lastCrossedPointIndex --;
            return packageRelevantPoints();
        }

        return packagedPoints;
    }

    @Override
    public Translation2d getTranslationSpeeds(Pose2d robotPosition) {
        ChassisSpeeds speeds = getSpeeds(robotPosition);
        return new Translation2d(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond);
    }

    @Override
    public Rotation2d getRotationSpeeds(Pose2d robotPosition) {
        ChassisSpeeds speeds = getSpeeds(robotPosition);
        return Rotation2d.fromRadians(speeds.omegaRadiansPerSecond);
    }
}