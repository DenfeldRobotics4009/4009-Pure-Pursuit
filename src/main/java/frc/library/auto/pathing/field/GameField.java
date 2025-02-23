package frc.library.auto.pathing.field;

import java.io.IOException;
import java.util.ArrayList;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.Trajectory.State;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.library.auto.pathing.pathObjects.Path;

public class GameField extends AprilTagFieldLayout {
    FieldMirrorType fieldMirrorType;
    Field2d fieldWidget = new Field2d();
    /**
     * Creates a GameField object that describes the game field.
     * @param widthMeters Distance in meters from the blue coordinate origin X value (0) 
     * to the red coordinate origin X value.
     * @param heightMeters Distance in meters from the blue coordinate origin Y value (0) 
     * to the red coordinate origin Y value.
     * @param fieldMirrorType Whether the field is rotated about the center, or flipped 
     * across the centerline.
     * @throws IOException 
     */
    public GameField(AprilTagFieldLayout aprilTagFieldLayout, FieldMirrorType fieldMirrorType) throws IOException {
        // Copy the given layout, this is the closest wpi gives to a copy constructor.
        super(aprilTagFieldLayout.getTags(), aprilTagFieldLayout.getFieldLength(), aprilTagFieldLayout.getFieldWidth());
        this.fieldMirrorType = fieldMirrorType;
        SmartDashboard.putData("Field", fieldWidget);
    }

    /**
     * @return The method the field is flipped/rotated between the blue and red alliances.
     */
    public FieldMirrorType getFieldMirrorType() {
        return fieldMirrorType;
    }

    /**
     * Sets the position of the robot in the field widget.
     * @param position
     */
    public void setRobotPose(Pose2d position) {
        fieldWidget.setRobotPose(position);
    }

    /**
     * @return smartdashboard.Field2d widget of this game field.
     */
    public Field2d getField2d() {
        return fieldWidget;
    }

    /**
     * Draws the path on the field widget.
     * @param name Name of the object drawing.
     * @param path Path to draw.
     */
    public void drawPath(String name, Path path) {
        ArrayList<State> pointsAsStates = new ArrayList<State>();
        for (Pose2d point : path.getPose2ds()) {
            pointsAsStates.add(new State(0, 0, 0, point, 0));
        }
        fieldWidget.getObject(name).setTrajectory(new Trajectory(pointsAsStates));
    }
}
