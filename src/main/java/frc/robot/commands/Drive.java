// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Controls;
import frc.robot.subsystems.Drivetrain;

public class Drive extends Command {

  Drivetrain drivetrain;
  Controls controls = Controls.getInstance();

  public Drive(Drivetrain drivetrain) {
    addRequirements(drivetrain);
    this.drivetrain = drivetrain;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    drivetrain.drive(
      ChassisSpeeds.fromFieldRelativeSpeeds(
        controls.getDriveForward() * Constants.Drivetrain.maxSpeedMetersPerSecond, 
        controls.getDriveLateral() * Constants.Drivetrain.maxSpeedMetersPerSecond, 
        controls.getDriveTurn() * Constants.Drivetrain.maxSpeedRadPerSecond, 
        drivetrain.position.getRotation()
      )
    );
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
