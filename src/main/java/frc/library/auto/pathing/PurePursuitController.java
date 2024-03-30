// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.library.auto.pathing;

import java.util.ArrayList;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.IntegerPublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.library.auto.pathing.controllers.RotationController;
import frc.library.auto.pathing.controllers.TranslationController;
import frc.library.auto.pathing.pathObjects.Path;
import frc.library.auto.pathing.pathObjects.PathPoint;
import frc.library.auto.pathing.pathObjects.PathState;

public class PurePursuitController extends Command implements RotationController, TranslationController {
    
    private class PurePursuitDiagnostics {
        DoublePublisher goalX;
        DoublePublisher goalY;
        DoublePublisher goalTheta;

        DoublePublisher currentX;
        DoublePublisher currentY;
        DoublePublisher currentTheta;

        DoublePublisher stateSpeed;
        IntegerPublisher lastCrossedPointIndex;

        DoublePublisher distanceToGoal;
        DoublePublisher percentAlongLine;
        DoublePublisher lookAhead;

        public PurePursuitDiagnostics() {
            NetworkTable purePursuitTable = NetworkTableInstance.getDefault().getTable("PurePursuit");
            goalX = purePursuitTable.getDoubleTopic("goalX").getEntry(0);
            goalY = purePursuitTable.getDoubleTopic("goalY").getEntry(0);
            goalTheta = purePursuitTable.getDoubleTopic("goalTheta").getEntry(0);

            currentX = purePursuitTable.getDoubleTopic("currentX").getEntry(0);
            currentY = purePursuitTable.getDoubleTopic("currentY").getEntry(0);
            currentTheta = purePursuitTable.getDoubleTopic("currentTheta").getEntry(0);

            stateSpeed = purePursuitTable.getDoubleTopic("stateSpeed").getEntry(0);
            lastCrossedPointIndex = purePursuitTable.getIntegerTopic("lastCrossedPointIndex").getEntry(0);

            distanceToGoal = purePursuitTable.getDoubleTopic("distanceToGoal").getEntry(0);
            percentAlongLine = purePursuitTable.getDoubleTopic("percentAlongLine").getEntry(0);
            lookAhead = purePursuitTable.getDoubleTopic("lookAhead").getEntry(0);
        }

        public void publishEntry(
            Pose2d goalPosition, 
            Pose2d currentPosition, 
            double speed, 
            int lastCrossedPointIndex,
            double percentAlongLine,
            double lookAhead
        ) {
            goalX.set(goalPosition.getX());
            goalY.set(goalPosition.getY());
            goalTheta.set(goalPosition.getRotation().getRadians());

            currentX.set(currentPosition.getX());
            currentY.set(currentPosition.getY());
            currentTheta.set(currentPosition.getRotation().getRadians());

            stateSpeed.set(speed);
            this.lastCrossedPointIndex.set(lastCrossedPointIndex);

            distanceToGoal.set(goalPosition.getTranslation().getDistance(currentPosition.getTranslation()));
            this.percentAlongLine.set(percentAlongLine);
            this.lookAhead.set(lookAhead);
        }
    }

    // Set of processed points
    final Path path;

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

    PurePursuitDiagnostics diagnostics = new PurePursuitDiagnostics();

    double lastDistanceToGoal = lookAheadMeters;

    /**
     * Follows a given path
     * @param path
     */
    public PurePursuitController(Path path) {
        this.path = path;
        rotationController.enableContinuousInput(-Math.PI, Math.PI);
    }

    /**
     * Follows a given path
     * @param pathPoints
     */
    public PurePursuitController(PathPoint... pathPoints) {
        this(new Path(pathPoints));
    }

    /**
     * Follows a given path
     * @param pathPoints
     */
    public PurePursuitController(double lastPointTolerance, PathPoint... pathPoints) {
        this(new Path(lastPointTolerance, pathPoints));
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


    // Called when the command is initially scheduled.
    @Override
    /**
     * Print path data
     */
    public void initialize() {
        lastCrossedPointIndex = 0;
        lookAheadMeters = 0.4;
        System.out.println("--- Following path of points: ---");
        for (PathPoint point : path.points) {System.out.println(point.posMeters + "," + point.orientation);}
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

        // calculate distance to last point
        double distanceToLastPointMeters = getLastPoint().posMeters.getDistance(lastPosition.getTranslation());

        return (
            // If we have passed the second to last point
            lastCrossedPointIndex >= (path.points.size() - 2) && 
            distanceToLastPointMeters < path.lastPointTolerance
        );
    }

    private static double clampStateSpeed(double stateSpeed) {
        return MathUtil.clamp(stateSpeed, 0.01, maxVelocityMeters);
    }

    public static double calculateLookAhead(double speed) {
        double clampedSpeed = clampStateSpeed(speed);
        return MathUtil.clamp(clampedSpeed * lookAheadScalar, 0.1, 2);
    }

    ChassisSpeeds getSpeeds(Pose2d robotPosition) {
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
                gotoGoal = path.points.get(path.points.size()-1).posMeters;
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
                gotoGoal = lookAheadPointA.posMeters.interpolate(
                    lookAheadPointB.posMeters, 
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
            relevantPoints.get(0).posMeters, relevantPoints.get(1).posMeters, robotTranslation
        );

        // Calculate position along line AB via finding difference between line length, and distance to B
        double distanceMetersAlongAB = lengthAB - relevantPoints.get(1).posMeters.getDistance(perpendicularIntersectionAB);

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
                gotoGoal = path.points.get(path.points.size()-1).posMeters;
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
                gotoGoal = lookAheadPointA.posMeters.interpolate(
                    lookAheadPointB.posMeters, 
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

        Rotation2d interpolatedRotation = relevantPoints.get(0).orientation.interpolate(
            relevantPoints.get(1).orientation, percentAlongAB);

        double stateSpeed = 0;
        // If our speed is increasing, accelerate instantly,
        // if our speed is decreasing, limit the deceleration
        if (relevantPoints.get(0).speedMetersPerSecond < relevantPoints.get(1).speedMetersPerSecond) {
            stateSpeed = relevantPoints.get(1).speedMetersPerSecond;
        } else {
            stateSpeed = PathPoint.getAtLinearInterpolation(
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
                pointB.posMeters, pointC.posMeters, robotTranslation
            );
            Translation2d perpendicularIntersectionAB = PathPoint.findPerpendicularIntersection(
                pointA.posMeters, pointB.posMeters, robotTranslation
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
                DriverStation.reportWarning("Could not grab point at index " + lastCrossedPointIndex + i, e.getStackTrace());
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