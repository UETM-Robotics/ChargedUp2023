package frc.robot.Utilities;

import frc.robot.Utilities.Geometry.Vector2d;

public class ChassisVelocity {
    private final Vector2d translationalVelocity;
    private final double angularVelocity;

    public ChassisVelocity(Vector2d translationalVelocity, double angularVelocity) {
        this.translationalVelocity = translationalVelocity;
        this.angularVelocity = angularVelocity;
    }

    /**
     * Gets the translational velocity component of this chassis velocity.
     * <p>
     * The {@code x} component of the translational velocity is the robot's forward velocity (positive is forwards) and
     * the {@code y} component is the sideways velocity (positive is to the left).
     *
     * @return The translational velocity w.r.t. the robot.
     */
    public Vector2d getTranslationalVelocity() {
        return translationalVelocity;
    }

    /**
     * Gets the angular velocity component of this chassis velocity.
     * <p>
     * Positive velocities are counter-clockwise movement.
     *
     * @return The angular velocity.
     */
    public double getAngularVelocity() {
        return angularVelocity;
    }
}