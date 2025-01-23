package frc.library.auto.pathing.field;

import java.util.logging.Logger;

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

        // If the position is not in the bounds of the field, throw a warning
        if (!inField()) {
            Logger.getLogger(GameFieldPose.class.getName())
                .warning(position.toString() + " is not within game field boundaries.");
        }
    }

    /**
     * Checks if this position is within the bounds.
     * @param cornerA
     * @param cornerB
     * @return True if this position is within the bounds.
     */
    public boolean inBounds(Translation2d cornerA, Translation2d cornerB) {
        return getTranslation().equals(clamp(cornerA, cornerB, getTranslation()));
    }

    /**
     * @return True if this position is inside the field.
     */
    public boolean inField() {
        return inBounds(new Translation2d(), new Translation2d(field.getFieldLength(), field.getFieldWidth()));
    }

    /**
     * Clamps a given double input within the boundaries of a and b, without
     * caring if a or b is the maximum or minimum.
     * @param a
     * @param b
     * @param input
     * @return input clamped within a and b
     */
    public static double nonSpecifiedClamp(double a, double b, double input) {
        if (a > b) { // a is maximum
            if (input > a) {return a;}
            else if (input < b) {return b;}
        } else { // b is maximum
            if (input > b) {return b;}
            else if (input < a) {return a;}
        }

        return input;
    }

    /**
     * Clamps the position into the bounds.
     * @param cornerA
     * @param cornerB
     * @return New GameFieldPose within boundaries of corner A and B
     */
    public GameFieldPose clamp(Translation2d cornerA, Translation2d cornerB) {
        return new GameFieldPose(
            getField(),
            clamp(cornerA, cornerB, getTranslation()),
            getRotation()
        );
    }

    /**
     * Clamps an input translation2d into the boundaries specified by the corners
     * @param cornerA
     * @param cornerB
     * @param Input
     * @return Translation2d within boundaries of corner A and B
     */
    public static Translation2d clamp(Translation2d cornerA, Translation2d cornerB, Translation2d Input) {
        return new Translation2d(
            nonSpecifiedClamp(cornerA.getX(), cornerB.getX(), Input.getX()),
            nonSpecifiedClamp(cornerA.getY(), cornerB.getY(), Input.getY())
        );
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

    /**
     * Checks equality between this GameField and another object.
     * @param obj The other object.
     * @return Whether the two objects are equal or not.
     */
    @Override
    public boolean equals(Object obj) {
        if (obj instanceof GameField) {
            // Compare poses
            return super.equals(obj);
        }
        
        return this == obj;
    }
}
