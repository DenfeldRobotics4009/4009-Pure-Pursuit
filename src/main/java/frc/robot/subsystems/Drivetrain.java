// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.library.auto.pathing.DriveSubsystem;

public class Drivetrain extends SubsystemBase implements DriveSubsystem {

  public Pose2d position = new Pose2d();
  public ChassisSpeeds velocity = new ChassisSpeeds();

  Timer velocityTimer = new Timer();

  Field2d field = new Field2d();

  static Drivetrain instance;

  public static Drivetrain getInstance() {
    if (instance == null) {
      instance = new Drivetrain();
    }

    return instance;
  }
  
  /**
   * A drivetrain purely intended for simulation and testing.
   */
  private Drivetrain() {
    velocityTimer.start();
    SmartDashboard.putData("Field", field);
  }

  @Override
  public void periodic() {

    double lastFrameTime = velocityTimer.get();
    // Restart timer to track next frame
    velocityTimer.restart();

    // Increment the position based on the last inputted velocity
    position = position.plus(
      new Transform2d(
        velocity.vxMetersPerSecond * lastFrameTime,
        velocity.vyMetersPerSecond * lastFrameTime,
        Rotation2d.fromRadians(velocity.omegaRadiansPerSecond * lastFrameTime)
      )
    );

    field.setRobotPose(position);
  }

  @Override
  public void drive(ChassisSpeeds speeds) {
    velocity = speeds;
  }

  @Override
  public Pose2d getPosition() {
    return position;
  }

  @Override
  public void setPosition(Pose2d position) {
    this.position = position;
  }
}
