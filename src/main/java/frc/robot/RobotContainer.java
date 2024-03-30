// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.library.auto.pathing.FollowControllers;
import frc.library.auto.pathing.PurePursuitController;
import frc.library.auto.pathing.pathObjects.Path;
import frc.library.auto.pathing.pathObjects.PathPoint;
import frc.robot.commands.Drive;
import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

  Drivetrain drivetrain = Drivetrain.getInstance();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
    drivetrain.setDefaultCommand(new Drive(drivetrain));

    PurePursuitController.setLookAheadScalar(1);
    PurePursuitController.setMaxVelocityMeters(5.08);
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return new RepeatCommand(
      new SequentialCommandGroup(
        new FollowControllers(
          new PurePursuitController(
            0.1,
            new PathPoint(
              new Translation2d(1, 1),
              Rotation2d.fromDegrees(0),
              1
            ),
            new PathPoint(
              new Translation2d(2, 5),
              Rotation2d.fromDegrees(0),
              2
            ),
            new PathPoint(
              new Translation2d(3, 1),
              Rotation2d.fromDegrees(0),
              3
            ),
            new PathPoint(
              new Translation2d(4, 5),
              Rotation2d.fromDegrees(0),
              4
            ),
            new PathPoint(
              new Translation2d(5, 1),
              Rotation2d.fromDegrees(0),
              5
            ),
            new PathPoint(
              new Translation2d(6, 5),
              Rotation2d.fromDegrees(0),
              5
            )
          ), 
          drivetrain
        ),

        new FollowControllers(
          new PurePursuitController(
            0.1,
            new PathPoint(
              new Translation2d(6, 5),
              Rotation2d.fromDegrees(0),
              5
            ),
            new PathPoint(
              new Translation2d(5, 1),
              Rotation2d.fromDegrees(180),
              4
            ),
            new PathPoint(
              new Translation2d(4, 5),
              Rotation2d.fromDegrees(360),
              3
            ),
            new PathPoint(
              new Translation2d(3, 1),
              Rotation2d.fromDegrees(0),
              2
            ),
            new PathPoint(
              new Translation2d(2, 5),
              Rotation2d.fromDegrees(180),
              1
            ),
            new PathPoint(
              new Translation2d(1, 1),
              Rotation2d.fromDegrees(360),
              0
            )
          ), 
          drivetrain
        )
      )
    );
  }
}
