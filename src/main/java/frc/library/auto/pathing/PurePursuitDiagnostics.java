// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.library.auto.pathing;

import java.util.ArrayList;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.Trajectory.State;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.IntegerPublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.library.auto.pathing.pathObjects.Path;

public class PurePursuitDiagnostics {
    DoublePublisher goalX;
    DoublePublisher goalY;
    DoublePublisher goalTheta;

    DoublePublisher currentX;
    DoublePublisher currentY;
    DoublePublisher currentTheta;

    DoublePublisher stateSpeed;
    IntegerPublisher lastCrossedPointIndex;

    DoublePublisher distanceToGoal;
    DoublePublisher percentAlongLine;
    DoublePublisher lookAhead;

    public static Field2d field = new Field2d();

    public PurePursuitDiagnostics(Path path) {
        NetworkTable purePursuitTable = NetworkTableInstance.getDefault().getTable("PurePursuit");
        goalX = purePursuitTable.getDoubleTopic("goalX").getEntry(0);
        goalY = purePursuitTable.getDoubleTopic("goalY").getEntry(0);
        goalTheta = purePursuitTable.getDoubleTopic("goalTheta").getEntry(0);

        currentX = purePursuitTable.getDoubleTopic("currentX").getEntry(0);
        currentY = purePursuitTable.getDoubleTopic("currentY").getEntry(0);
        currentTheta = purePursuitTable.getDoubleTopic("currentTheta").getEntry(0);

        stateSpeed = purePursuitTable.getDoubleTopic("stateSpeed").getEntry(0);
        lastCrossedPointIndex = purePursuitTable.getIntegerTopic("lastCrossedPointIndex").getEntry(0);

        distanceToGoal = purePursuitTable.getDoubleTopic("distanceToGoal").getEntry(0);
        percentAlongLine = purePursuitTable.getDoubleTopic("percentAlongLine").getEntry(0);
        lookAhead = purePursuitTable.getDoubleTopic("lookAhead").getEntry(0);

        SmartDashboard.putData("Field", field);

        ArrayList<State> pointsAsStates = new ArrayList<State>();
        for (Pose2d point : path.getPose2ds()) {
            pointsAsStates.add(new State(0, 0, 0, point, 0));
        }

        field.getObject("Path Trajectory").setTrajectory(
            new Trajectory(pointsAsStates)
        );
    }

    public void publishEntry(
        Pose2d goalPosition, 
        Pose2d currentPosition, 
        double speed, 
        int lastCrossedPointIndex,
        double percentAlongLine,
        double lookAhead
    ) {
        goalX.set(goalPosition.getX());
        goalY.set(goalPosition.getY());
        goalTheta.set(goalPosition.getRotation().getRadians());

        currentX.set(currentPosition.getX());
        currentY.set(currentPosition.getY());
        currentTheta.set(currentPosition.getRotation().getRadians());

        stateSpeed.set(speed);
        this.lastCrossedPointIndex.set(lastCrossedPointIndex);

        distanceToGoal.set(goalPosition.getTranslation().getDistance(currentPosition.getTranslation()));
        this.percentAlongLine.set(percentAlongLine);
        this.lookAhead.set(lookAhead);

        field.setRobotPose(currentPosition);
    }
}
