package frc.robot.Utilities;

import edu.wpi.first.math.geometry.Rotation2d;
//import frc.robot.Utilities.Swerve.SwerveModuleState;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public class CTREModuleState
{
    public static SwerveModuleState optimize(SwerveModuleState desired, Rotation2d curAngle)
    {
        double targetAngle = putInScope360(curAngle.getDegrees(), desired.angle.getDegrees());
        double targetSpeed =desired.speedMetersPerSecond;
        double delta = targetAngle - curAngle.getDegrees();

        if(Math.abs(delta) > 90)
        {
            targetSpeed *= -1;
            targetAngle = delta > 90 ? (targetAngle -= 180) : (targetAngle += 180);
        }

        return new SwerveModuleState(targetSpeed, Rotation2d.fromDegrees(targetAngle));
    }

    private static double putInScope360(double scopeReference, double newAngle)
    {
        double lb, ub;
        double lowerOffset = scopeReference % 360;

        if(lowerOffset >= 0)
        {
            lb = scopeReference - lowerOffset;
            ub = scopeReference + (360 - lowerOffset);
        }

        else
        {
            ub = scopeReference - lowerOffset;
            lb = scopeReference + (360 - lowerOffset);
        }

        while(newAngle < lb)
        {
            newAngle += 360;
        }

        while(newAngle > ub)
        {
            newAngle -= 360;
        }

        if(newAngle - scopeReference >180)
        {
            newAngle -= 360;
        }

        else if(newAngle - scopeReference < -180)
        {
            newAngle += 360;
        }

        return newAngle;
    }
}
