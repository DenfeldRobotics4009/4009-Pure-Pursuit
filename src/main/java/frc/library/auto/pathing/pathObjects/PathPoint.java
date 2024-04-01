// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.library.auto.pathing.pathObjects;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.library.auto.pathing.PurePursuitController;

/**
 * A point along a Path
 */
public class PathPoint extends Pose2d {

    public double speedMetersPerSecond; // Corrected by path constructor

    /**
     * Constructs a new PathPoint with the given statistics. The
     * order of PathPoints constructed will define the order they
     * are driven by the robot.
     * @param poseMeters Position of point on the field in meters
     * @param orientation The goal orientation of the robot when it reaches this point
     * @param speedMetersPerSecond The speed the robot should travel THROUGH this point
     */
    public PathPoint(Translation2d poseMeters, Rotation2d orientation, double speedMetersPerSecond) {
        super(poseMeters, orientation);

        // May be overridden
        this.speedMetersPerSecond = MathUtil.clamp(speedMetersPerSecond, 0, PurePursuitController.maxVelocityMeters);
    }

    /**
     * Constructs a new PathPoint with the given statistics. The
     * order of PathPoints constructed will define the order they
     * are driven by the robot.
     * @param posMeters Position of point on the field in meters
     * @param speedMetersPerSecond The speed the robot should travel THROUGH this point
     */
    public PathPoint(Pose2d poseMeters, double speedMetersPerSecond) {
        this(poseMeters.getTranslation(), poseMeters.getRotation(), speedMetersPerSecond);
    }
    /**
     * 
     * @param Initial Initial Value
     * @param Final Final Value
     * @param PercentBetween Position (In Percent) along line starting from 0
     * 
     * @return Calculated value between Initial and Final 
     * that is on the interpolated line function.
     */
    public static double interpolate(
        double Initial, double Final, double PercentBetween
    ) {
        return (Final - Initial) * PercentBetween + Initial;
    }
    
    /**
     * Returns the coordinates of the intersection between this line,
     * and a perpendicular line that coincides with point Source.
     * @param source Line to attach perpendicular line to
     * @return Coordinates of intersection between lines
     */
    public static Translation2d findPerpendicularIntersection(Translation2d pointA, Translation2d pointB, Translation2d source) {

        double deltaX = pointB.getX() - pointA.getX();
        double deltaY = pointB.getY() - pointA.getY();

        Translation2d intersection;

        // Check for edge cases, deltaX being 0 or deltaY being 0

        if (deltaY == 0) {
            //PathFollower.println("Calculating perpendicular intersection from horizontal line");

            intersection = new Translation2d(
                source.getX(), pointA.getY()
            );

        } else if (deltaX == 0) {
            //PathFollower.println("Calculating perpendicular intersection from vertical line");

            intersection = new Translation2d(
                pointA.getX(), source.getY()
            );

        } else {

            //PathFollower.println("Calculating perpendicular intersection from sloped line");

            double Slope = deltaY / deltaX;

            double xIntercept = (
                    (source.getX() / (Slope*Slope)) 
                    + (source.getY() / (Slope)) 
                    - (pointA.getY() / (Slope))
                    + (pointA.getX())
                ) / (1 + 1/(Slope*Slope));

            intersection = new Translation2d(
                xIntercept, 
                Slope*(xIntercept-pointA.getX()) + pointA.getY()
            );
        }
        
        //PathFollower.println("Found perpendicular intersection at " + intersection);
        return clamp(pointA, pointB, intersection);
    }

    /**
     * Clamps an input translation2d into the boundaries specified by the corners
     * @param cornerA
     * @param cornerB
     * @param Input
     * @return Translation2d within boundaries of corner A and B
     */
    public static Translation2d clamp(Translation2d cornerA, Translation2d cornerB, Translation2d Input) {
        return new Translation2d(
            nonSpecifiedClamp(cornerA.getX(), cornerB.getX(), Input.getX()),
            nonSpecifiedClamp(cornerA.getY(), cornerB.getY(), Input.getY())
        );
    }

    /**
     * Constructs a new pathPathPoint via
     * interpolating values from this to
     * finalPoint.
     * @param finalPoint Ending point
     * @param t normalized distance from initial point
     * @return new PathPoint
     */
    public PathPoint interpolate(PathPoint finalPoint, double t) {
        return new PathPoint(
            // Position
            getTranslation().interpolate(finalPoint.getTranslation(), t), 
            // Rotation
            getRotation().interpolate(finalPoint.getRotation(), t),
            // Speed (Unadjusted)
            interpolate(
                speedMetersPerSecond, finalPoint.speedMetersPerSecond, t)
        );
    }

    public double getDistance(Pose2d pathPoint) {
        return getTranslation().getDistance(pathPoint.getTranslation());
    }

    /**
     * Clamps a given double input within the boundaries of a and b, without
     * caring if a or b is the maximum or minimum.
     * @param a
     * @param b
     * @param input
     * @return input clamped within a and b
     */
    public static double nonSpecifiedClamp(double a, double b, double input) {
        if (a > b) { // a is maximum
            if (input > a) {return a;}
            else if (input < b) {return b;}
        } else { // b is maximum
            if (input > b) {return b;}
            else if (input < a) {return a;}
        }

        return input;
    }
} 
