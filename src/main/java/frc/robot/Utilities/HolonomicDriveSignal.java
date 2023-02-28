package frc.robot.Utilities;

import frc.robot.Utilities.Geometry.Vector2d;

public class HolonomicDriveSignal {
    private final Vector2d translation;
    private final double rotation;
    private final boolean fieldOriented;

    public HolonomicDriveSignal(Vector2d translation, double rotation, boolean fieldOriented) {
        this.translation = translation;
        this.rotation = rotation;
        this.fieldOriented = fieldOriented;
    }

    public static HolonomicDriveSignal NEUTRAL = new HolonomicDriveSignal(new Vector2d(0, 0), 0, false);

    public Vector2d getTranslation() {
        return translation;
    }

    public double getRotation() {
        return rotation;
    }

    public boolean isFieldOriented() {
        return fieldOriented;
    }
}