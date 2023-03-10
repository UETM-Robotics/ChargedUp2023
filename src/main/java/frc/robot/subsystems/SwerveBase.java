/*package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
//import edu.wpi.first.wpilibj.ki;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Utilities.Controllers;
import frc.robot.Utilities.Constants.TechConstants;
import frc.robot.Utilities.Drivers.Swerve.SwerveModule;

public class SwerveBase extends SubsystemBase
{
    private final SwerveModule frontRight = Controllers.getInstance().getRightFrontSwerveModuleU();
    private final SwerveModule frontLeft = Controllers.getInstance().getLeftFrontSwerveModuleU();
    private final SwerveModule backRight = Controllers.getInstance().getRightHindSwerveModuleU();
    private final SwerveModule backLeft = Controllers.getInstance().getLeftHindSwerveModuleU();

    private final AHRS gyro = Controllers.getInstance().getGyro();

    public SwerveBase()
    {
        SmartDashboard.putString("SWERVE", "INSTANTIATING");        

        new Thread(() -> {
            try{
                Thread.sleep(1000);
                zeroHeading();
            } catch(Exception e){}
        }).start();
    }

    public void zeroHeading()
    {
        gyro.reset();
    }

    public double getHeading()
    {
        return Math.IEEEremainder(gyro.getAngle(), 360);
    }

    public Rotation2d geRotation2d()
    {
        return Rotation2d.fromDegrees(getHeading());
    }

    @Override
    public void periodic()
    {
        SmartDashboard.putNumber("ROBOT HEADING", getHeading());
    }

    public void stopModules()
    {
        frontLeft.stop();
        frontRight.stop();
        backLeft.stop();
        backRight.stop();
    }

    public void setModuleStates(SwerveModuleState[] states)
    {
        //SwerveDriveKinematics.desaturateWheelSpeeds(states, null, getHeading(), getHeading(), getHeading());
        SwerveDriveKinematics.desaturateWheelSpeeds(states, TechConstants.kDriveMaxVelocity);
        frontLeft.setDesiredState(states[0]);
        frontRight.setDesiredState(states[1]);
        backLeft.setDesiredState(states[2]);
        backRight.setDesiredState(states[3]);
    }
}
*/