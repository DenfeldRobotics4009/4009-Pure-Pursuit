// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;

/** Add your docs here. */
public class Controls {

    public Joystick driveController = new Joystick(0);

    static Controls instance;

    public static Controls getInstance() {
        if (instance == null) {
            instance = new Controls();
        }

        return instance;
    }

    private Controls() {

    }

    public double getDriveForward() {
        return deadband(-driveController.getRawAxis(1), Constants.DriverConstants.driveDeadBandPercent);
    }

    public double getDriveLateral() {
        return deadband(-driveController.getRawAxis(0), Constants.DriverConstants.driveDeadBandPercent);
    }

    public double getDriveTurn() {
        return deadband(-driveController.getRawAxis(2), Constants.DriverConstants.steerDeadBandPercent);
    }

    private static double deadband(double value, double deadband) {
        if( Math.abs(value) < deadband ) {
            return 0;
        } else{
            return (value + (-Math.copySign(1, value) * deadband)) / (1 - deadband);
        }
    }

}
