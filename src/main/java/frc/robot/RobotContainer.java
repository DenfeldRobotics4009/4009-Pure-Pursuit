// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.library.auto.pathing.FollowControllers;
import frc.library.auto.pathing.PurePursuitController;
import frc.library.auto.pathing.PurePursuitSettings;
import frc.library.auto.pathing.SetDrivePosition;
import frc.library.auto.pathing.field.FieldMirrorType;
import frc.library.auto.pathing.field.GameField;
import frc.library.auto.pathing.pathObjects.Path;
import frc.robot.commands.Drive;
import frc.robot.subsystems.Drivetrain;

import java.io.IOException;
import java.util.EnumSet;

import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.NetworkTableEvent;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PrintCommand;
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

  SendableChooser<SequentialCommandGroup> autoChooser = new SendableChooser<SequentialCommandGroup>();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
    drivetrain.setDefaultCommand(new Drive(drivetrain));

    GameField gameField = null;
    try {
      gameField = new GameField(AprilTagFields.k2023ChargedUp.loadAprilTagLayoutField(), FieldMirrorType.Mirrored);
    } catch (IOException e) {
      // AprilTagFields file not found
      e.printStackTrace();
    }

    PurePursuitSettings config = new PurePursuitSettings(gameField, Alliance.Blue)
      .setLookAheadScalar(0.2)
      .setDistanceToGoalTolerance(0.1)
      .setDefaultEndpointTolerance(0.1);

    PurePursuitController pathA = new PurePursuitController(Path.getFromPathPlanner(config, Alliance.Blue, "ExamplePathInitial"));
    SequentialCommandGroup autoCommand = new SequentialCommandGroup(
      new SetDrivePosition(drivetrain, pathA.getPath().getStartingPoseSupplier()),
      new FollowControllers(pathA, drivetrain),
      new PrintCommand("Staging second path"),
      new FollowControllers(new PurePursuitController(Path.getFromPathPlanner(config, Alliance.Blue, "ExamplePathFinal")), drivetrain)
    );

    autoChooser.addOption("Example Auto", autoCommand);
    SmartDashboard.putData(autoChooser);
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
    return autoChooser.getSelected();
  }
}
