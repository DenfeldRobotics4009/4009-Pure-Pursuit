// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.library.auto.pathing;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;

public class SetDrivePosition extends Command {
  DriveSubsystem driveSubsystem;
  Supplier<Pose2d> position;
  /** Creates a new SetDrivePosition. */
  public SetDrivePosition(DriveSubsystem driveSubsystem, Supplier<Pose2d> position) {
    this.driveSubsystem = driveSubsystem;
    this.position = position;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    driveSubsystem.setPosition(position.get());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
