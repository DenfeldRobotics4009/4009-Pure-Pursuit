// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.library.auto.pathing;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.library.auto.pathing.field.FieldMirrorType;
import frc.library.auto.pathing.field.GameField;

public class PurePursuitSettings {

    // The distance the robot must be within from the last point
    public double endpointTolerance = 0.4;
    // The front of the robot
    public Rotation2d forwardAngle = new Rotation2d();
    // LookAhead distance = lookAheadScalar * speed
    public double lookAheadScalar = 1.5;
    // The maximum speed the robot can travel
    public double maxVelocityMeters = 5.06; // Mk4I Swerve Module
    // The maximum speed at which the robot can accelerate, however,
    // this metric is only used in the case of decelerating along the path.
    public double maxAccelerationMeters = 2.7;
    public double turningP = 5, turningI = 0, turningD = 0;
    // If the robot comes this close to its goal, it will increment 
    // the last crossed point index.
    public double distanceToGoalTolerance = endpointTolerance * 0.5;    
    // The method of which paths are flipped.
    public FieldMirrorType fieldMirrorType = FieldMirrorType.Mirrored;

    public GameField field;
    public Alliance originAlliance;

    /**
     * Creates an object detailing the settings of the pure pursuit controller
     * and its paths. The parameters are required settings.
     * @param field The field of the current game year.
     * @param pathOriginAlliance The default alliance the paths are built for. This
     * can also be changed per path.
     */
    public PurePursuitSettings(GameField field, Alliance pathOriginAlliance) {
        this.field = field;
        this.originAlliance = pathOriginAlliance;
    }

    /**
     * Configures the PID components of the steering
     * PID controller.
     * @param P
     * @param I
     * @param D
     * @return This for chaining.
     */
    public PurePursuitSettings setTurningPID(double P, double I, double D) {
        this.turningP = P;
        this.turningI = I;
        this.turningD = D;
        return this;
    }

    /**
     * Sets the field the pure pursuit controller is built for.
     * @param field
     * @return This for chaining.
     */
    public PurePursuitSettings setField(GameField field) {
        this.field = field;
        return this;
    }

    /**
     * Sets the alliance paths are built for by default.
     * @param alliance
     * @return This for chaining.
     */
    public PurePursuitSettings setAllianceOrigin(Alliance alliance) {
        this.originAlliance = alliance;
        return this;
    }

    /**
     * Sets the maximum allowed deceleration along paths
     * This does not limit acceleration, only deceleration.
     * @param maxAccelerationMeters double [0, 10]
     * @return This for chaining.
     */
    public PurePursuitSettings setMaxAccelerationMeters(double maxAccelerationMeters) {
        this.maxAccelerationMeters = 
            MathUtil.clamp(maxAccelerationMeters, 0, 10);
        return this;
    }

    /**
     * Sets the maximum allowed speed of paths
     * @param maxVelocityMeters double [0, 10]
     * @return This for chaining.
     */
    public PurePursuitSettings setMaxVelocityMeters(double maxVelocityMeters) {
        this.maxVelocityMeters = 
            MathUtil.clamp(maxVelocityMeters, 0, 10);
        return this;
    }

    /**
     * Sets the multiplier used to calculate lookahead
     * from the current speed of the robot along a path.
     * lookAheadMeters = speedMeters * lookAheadScalar
     * @param lookAheadScalar double, [0.1, 10]
     * @return This for chaining.
     */
    public PurePursuitSettings setLookAheadScalar(double lookAheadScalar) {
        this.lookAheadScalar = 
            MathUtil.clamp(lookAheadScalar, 0.1, 10);
        return this;
    }

    /**
     * Sets the angle offset paths will be adjusted by,
     * allowing what is considered the front of the robot
     * to be changed.
     * @param angle Rotation2d
     * @return This for chaining.
     */
    public PurePursuitSettings setForwardAngle(Rotation2d forwardAngle) {
        this.forwardAngle = forwardAngle;
        return this;
    }

    /**
     * Sets the default allowed distance from the robot to the
     * last point in the path for the path command to end.
     * @param tolerance
     * @return This for chaining.
     */
    public PurePursuitSettings setDefaultEndpointTolerance(double tolerance) {
        this.endpointTolerance = MathUtil.clamp(tolerance, 0, 1);
        return this;
    }

    /**
     * Sets the default allowed distance from the robot to its
     * goal before incrementing last crossed point index
     * @param tolerance
     * @return This for chaining.
     */
    public PurePursuitSettings setDistanceToGoalTolerance(double tolerance) {
        this.distanceToGoalTolerance = MathUtil.clamp(tolerance, 0, 1);
        return this;
    }

    /**
     * Sets the method that paths are flipped to the opposite alliance.
     * @param fieldMirrorType
     * @return This for chaining.
     */
    public PurePursuitSettings setFieldMirrorType(FieldMirrorType fieldMirrorType) {
        this.fieldMirrorType = fieldMirrorType;
        return this;
    }
}
