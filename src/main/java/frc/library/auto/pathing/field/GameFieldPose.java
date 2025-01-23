package frc.library.auto.pathing.field;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public class GameFieldPose extends Pose2d {
    GameField field;

    /**
     * Creates a position on the field.
     * @param field The game field.
     * @param position The position of the point.
     * @param rotation The rotation of the point.
     */
    public GameFieldPose(GameField field, Translation2d position, Rotation2d rotation) {
        super(position, rotation);
        this.field = field;
    }

    /**
     * Flips the point to the corresponding coordinate position for the opposite alliance.
     * @param field The game field specifying the dimensions and mirror type.
     * @return A new point on the opposite side of the field.
     */
    public GameFieldPose flipAllianceOrigin() {
        // If field is mirrored across centerline
        if (field.getFieldMirrorType().equals(FieldMirrorType.Mirrored)) {
            return new GameFieldPose(
                field,
                new Translation2d(field.getFieldLength() - getX(), getY()), // Flip X axis
                Rotation2d.fromDegrees(180).minus(getRotation())            // Reverse rotation
            );
        // If field is rotated about centerline
        } else if (field.getFieldMirrorType().equals(FieldMirrorType.Rotated)) {
            return new GameFieldPose(
                field,
                new Translation2d(field.getFieldLength() - getX(), field.getFieldWidth() - getY()), // Flip X and Y axis
                Rotation2d.fromDegrees(180).minus(getRotation())                                    // Reverse rotation
            );
        } else {
            // Incase another enum value is added
            throw new IllegalArgumentException("Field Mirror type must be mirrored or rotated");
        }
    }

    /**
     * @return The field this point is placed upon
     */
    public GameField getField() {
        return field;
    }
}
