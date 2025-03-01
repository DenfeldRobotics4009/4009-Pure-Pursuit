// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.library.auto.pathing;

import java.util.ArrayList;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.library.auto.pathing.controllers.RotationController;
import frc.library.auto.pathing.controllers.TranslationController;

/**
 * Drives the robot from one or two given commands that may
 * supply a rotational and translational speed.
 */
public class FollowControllers extends Command {

  DriveSubsystem driveSubsystem;

  TranslationController translationController;
  RotationController rotationController;

  // A list of all commands being ran under the drive command
  ArrayList<Command> commands = new ArrayList<>();

  final boolean race;

  /**
   * Drives the robot based upon each controllers speed
   * output. Each controller is expected to be a command, and will
   * be scheduled as this command initializes.
   * @param translationController
   * @param rotationController
   * @param race Whether the controller commands will race. If true,
   * when one controller ends, this command will end. If false, this
   * command will not end until both commands are finished. 
   */
  public FollowControllers(
    TranslationController translationController, 
    RotationController rotationController,
    boolean race,
    DriveSubsystem driveSubsystem
  ) {
    this.driveSubsystem = driveSubsystem;
    this.translationController = translationController;
    this.rotationController = rotationController;

    // If the supplied controllers are not commands, they will still function.
    if (translationController instanceof Command) {
      commands.add((Command) translationController);
    }

    if (rotationController instanceof Command) {
      commands.add((Command) rotationController);
    }

    this.race = race;

    addRequirements(driveSubsystem);
  }

  /**
   * Drives the robot based upon the controllers speed
   * output. The controller is expected to be a command, and will
   * be scheduled as this command initializes.
   * @param poseController
   * @param driveSubsystem
   */
  public <PoseController extends TranslationController & RotationController> 
    FollowControllers(PoseController poseController, DriveSubsystem driveSubsystem) 
  {
    this.driveSubsystem = driveSubsystem;
    this.translationController = poseController;
    this.rotationController = poseController;

    if (poseController instanceof Command) {
      commands.add((Command) poseController);
    }

    race = true;

    addRequirements(driveSubsystem);
  }

  @Override
  /**
   * Schedules the included command.
   */
  public final void initialize() {
    for (Command command : commands) {
      command.initialize();
    }
  }

  @Override
  /**
   * Sets the drive subsystem speeds/
   */
  public final void execute() {
    for (Command command : commands) {
      if (!command.isFinished()) command.execute();
    }
    Pose2d poseSpeeds = new Pose2d(
      translationController.getTranslationSpeeds(driveSubsystem.getPosition()), 
      rotationController.getRotationSpeeds(driveSubsystem.getPosition())
    );

    driveSubsystem.drive(
      poseSpeeds.getX(), poseSpeeds.getY(), poseSpeeds.getRotation().getRadians()
    );
  }

  @Override
  /**
   * Cancels all commands if they are still currently running
   */
  public final void end(boolean interrupted) {
    for (Command command : commands) {
      command.end(interrupted);
    }
  }

  boolean raceIsFinished() {
    for (Command command : commands) {
      if (command.isFinished()) {
        return true;
      }
    }

    return false;
  }

  boolean parallelIsFinished() {
    for (Command command : commands) {
      if (!command.isFinished()) {
        return false;
      }
    }

    return true;
  }

  @Override
  public final boolean isFinished() {
    if (race) {
      return raceIsFinished();
    } else {
      return parallelIsFinished();
    }
  }
}
