package frc.library.auto.pathing.field;

import java.io.IOException;
import java.nio.file.Path;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Translation2d;

public class GameField  {
    AprilTagFieldLayout aprilTagFieldLayout;
    FieldMirrorType fieldMirrorType;
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
    public GameField(AprilTagFields fieldLayoutResource, FieldMirrorType fieldMirrorType) throws IOException {
        this.aprilTagFieldLayout = fieldLayoutResource.loadAprilTagLayoutField();
        this.fieldMirrorType = fieldMirrorType;
    }

    public FieldMirrorType getFieldMirrorType() {
        return fieldMirrorType;
    }
    
    /**
     * Returns the length of the field the layout is representing in meters.
     *
     * @return width, in meters
     */
    public double getFieldWidth() {
        return aprilTagFieldLayout.getFieldWidth();
    }

    /**
     * Returns the length of the field the layout is representing in meters.
     *
     * @return length, in meters
     */
    public double getFieldLength() {
        return aprilTagFieldLayout.getFieldLength();
    }
}
