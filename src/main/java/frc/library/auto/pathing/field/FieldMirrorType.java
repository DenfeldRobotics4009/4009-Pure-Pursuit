package frc.library.auto.pathing.field;

/**
 * Describes how the game field is mirrored/rotated.
 * @see https://docs.wpilib.org/en/stable/docs/software/basic-programming/coordinate-system.html
 */
public enum FieldMirrorType {
    /**
     * The field is flipped along the centerline. Ie: 2023 field.
     */
    Mirrored,
    
    /**
     * The field is rotated about the center. Ie: 2022 field.
     */
    Rotated
}
