package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Utilities.Controllers;

public class Swerve extends SubsystemBase
{
    public SwerveDriveOdometry odometry;
    public SwerveModule[] mSwerveMods;
    public AHRS gyro;

    public Swerve()
    {
        gyro = Controllers.getInstance().getGyro();
        gyro.calibrate();
    }
}
