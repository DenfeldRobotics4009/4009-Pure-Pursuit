// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.library.auto.pathing;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;

public class PurePursuitSettings {
    private PurePursuitSettings() {}

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

    /**
     * Configures the PID components of the steering
     * PID controller.
     * @param P
     * @param I
     * @param D
     */
    public static void setTurningPID(double P, double I, double D) {
        PurePursuitSettings.turningP = P;
        PurePursuitSettings.turningI = I;
        PurePursuitSettings.turningD = D;
    }

    /**
     * Sets the maximum allowed deceleration along paths
     * This does not limit acceleration, only deceleration.
     * @param maxAccelerationMeters double [0, 10]
     */
    public static void setMaxAccelerationMeters(double maxAccelerationMeters) {
        PurePursuitSettings.maxAccelerationMeters = 
            MathUtil.clamp(maxAccelerationMeters, 0, 10);
    }

    /**
     * Sets the maximum allowed speed of paths
     * @param maxVelocityMeters double [0, 10]
     */
    public static void setMaxVelocityMeters(double maxVelocityMeters) {
        PurePursuitSettings.maxVelocityMeters = 
            MathUtil.clamp(maxVelocityMeters, 0, 10);
    }

    /**
     * Sets the multiplier used to calculate lookahead
     * from the current speed of the robot along a path.
     * lookAheadMeters = speedMeters * lookAheadScalar
     * @param lookAheadScalar double, [0.1, 10]
     */
    public static void setLookAheadScalar(double lookAheadScalar) {
        PurePursuitSettings.lookAheadScalar = 
            MathUtil.clamp(lookAheadScalar, 0.1, 10);
    }

    /**
     * Sets the angle offset paths will be adjusted by,
     * allowing what is considered the front of the robot
     * to be changed.
     * @param angle Rotation2d
     */
    public static void setForwardAngle(Rotation2d forwardAngle) {
        PurePursuitSettings.forwardAngle = forwardAngle;
    }

    /**
     * Sets the default allowed distance from the robot to the
     * last point in the path for the path command to end.
     * @param tolerance
     */
    public static void setDefaultEndpointTolerance(double tolerance) {
        PurePursuitSettings.endpointTolerance = MathUtil.clamp(tolerance, 0, 1);
    }

    /**
     * Sets the default allowed distance from the robot to its
     * goal before incrementing last crossed point index
     * @param tolerance
     */
    public static void setDistanceToGoalTolerance(double tolerance) {
        PurePursuitSettings.distanceToGoalTolerance = MathUtil.clamp(tolerance, 0, 1);
    }
}
