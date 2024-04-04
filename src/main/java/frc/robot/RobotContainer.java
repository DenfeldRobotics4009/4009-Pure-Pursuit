// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.library.auto.pathing.FollowControllers;
import frc.library.auto.pathing.PurePursuitController;
import frc.library.auto.pathing.PurePursuitSettings;
import frc.library.auto.pathing.SetDrivePosition;
import frc.library.auto.pathing.pathObjects.Path;
import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
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

    PurePursuitController pathA = new PurePursuitController(Path.getFromPathPlanner("7PieceFirstNote"));
    SequentialCommandGroup autoCommand = new SequentialCommandGroup(
      new InstantCommand(
        () -> {
          PurePursuitSettings.setLookAheadScalar(0.2);
          PurePursuitSettings.setDistanceToGoalTolerance(0.1);
        }
      ),
      new SetDrivePosition(drivetrain, pathA.getPath().getStartingPose()),
      new FollowControllers(pathA, drivetrain),
      new FollowControllers(new PurePursuitController(Path.getFromPathPlanner("7PieceSecondNote")), drivetrain),
      new FollowControllers(new PurePursuitController(Path.getFromPathPlanner("7PieceThirdNote")), drivetrain),
      new FollowControllers(new PurePursuitController(Path.getFromPathPlanner("7PieceFourthNote")), drivetrain),
      new FollowControllers(new PurePursuitController(Path.getFromPathPlanner("7PieceFifthNote")), drivetrain),
      new FollowControllers(new PurePursuitController(Path.getFromPathPlanner("7PieceSixthNote")), drivetrain)
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
