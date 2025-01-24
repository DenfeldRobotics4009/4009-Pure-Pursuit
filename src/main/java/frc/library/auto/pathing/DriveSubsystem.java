// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.library.auto.pathing;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Subsystem;

/**
 * A generic drive subsystem format utilized by
 * the autonomous pathing algorithm, and implemented
 * by the user.
 */
public interface DriveSubsystem extends Subsystem {
    
    /**
     * Drives the robot at the given field oriented direction.
     * @param xMetersPerSecond Meters per second in the positive x direction. (To the left of the driver)
     * @param yMetersPerSecond Meters per second in the positive y direction. (Downfield)
     * @param radPerSecond Turning radians per second.
     */
    void drive(double xMetersPerSecond, double yMetersPerSecond, double radPerSecond);

    /**
     * @return the current most accurate field
     * oriented position of the robot in meters.
     */
    Pose2d getPosition();

    /**
     * Sets the position of the supplied drive
     * subsystem.
     * @param position Pose2d
     */
    void setPosition(Pose2d position);
}
