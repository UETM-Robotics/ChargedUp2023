package frc.robot.subsystems;

import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.ctre.phoenix.sensors.SensorTimeBase;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Utilities.CTREModuleState;
import frc.robot.Utilities.Controllers;
import frc.robot.Utilities.Constants.TechConstants;
import frc.robot.Utilities.Drivers.SparkHelper;
import frc.robot.Utilities.Drivers.SparkMaxU;

public class SwerveModule
{
    private SparkMaxU mDrive, mAngle;
    private Rotation2d angleOffset, lastAngle;
    private CANCoder canCoder;
    private boolean canReversed;
    private boolean angInverted;
    
    public SwerveModule(SparkMaxU mDrive, SparkMaxU mAngle, CANCoder canCoder, boolean canReversed, boolean angInverted)
    {
        this.mDrive = mDrive;
        this.mAngle = mAngle;
        this.canCoder = canCoder;

        this.canReversed = canReversed;
        this.angInverted = angInverted;

        SparkHelper.setPIDGains(this.mDrive, 0, TechConstants.kDriveVelocityKp, TechConstants.kDriveVelocityKi, TechConstants.kDriveVelocityKd, TechConstants.kDriveVelocityKf, TechConstants.kDriveVelocityRampRate, TechConstants.kDriveVelocityIZone);

        lastAngle = getState().angle;
    }

    public void setDesiredState(SwerveModuleState desired, boolean isOpenLoop)
    {
        desired = CTREModuleState.optimize(desired, getState().angle);

        setAngle(desired);
        setSpeed(desired, isOpenLoop);
    }

    private void setSpeed(SwerveModuleState desired, boolean isOpenLoop)
    {
        if(isOpenLoop)
        {
            double percentOut = desired.speedMetersPerSecond / TechConstants.kDriveMaxVelocity;
            mDrive.set(percentOut);
        }

        else
        {
            double vel = desired.speedMetersPerSecond / TechConstants.kDriveRPM_To_MetersPerSec;
            mDrive.set(vel, ControlType.kVelocity);
        }
    }

    private void setAngle(SwerveModuleState desired)
    {
        Rotation2d angle = Math.abs(desired.speedMetersPerSecond) <= (TechConstants.kDriveMaxVelocity * 0.01) ? lastAngle : desired.angle;

        //THIS MAY BE WRONG
        double degrees = desired.speedMetersPerSecond  / (360.0 / (150/7 * 42));

        mAngle.set(degrees);
        lastAngle = angle;
    }

    private Rotation2d getAngle()
    {
        //THIS MAY BE WRONG
        double degrees = mAngle.getEncoder().getPosition() * (360.0 / (150/7 * 42));

        return Rotation2d.fromDegrees(degrees);
    }

    public Rotation2d getCanCoder()
    {
        return Rotation2d.fromDegrees(canCoder.getAbsolutePosition());
    }

    public void resetToAbsolute()
    {
        double absPos =  getCanCoder().getDegrees() / (360.0 / (150/7 * 42));
        mAngle.getEncoder().setPosition(absPos);
    }

    private void configAngleEncoder()
    {
        canCoder.configFactoryDefault();

        canCoder.configAbsoluteSensorRange(AbsoluteSensorRange.Unsigned_0_to_360);
        canCoder.configSensorDirection(canReversed);
        canCoder.configSensorInitializationStrategy(SensorInitializationStrategy.BootToAbsolutePosition);
        canCoder.configFeedbackCoefficient(0.087890625, "deg", SensorTimeBase.PerSecond);
    }

    private void configAngleMotor()
    {
        mAngle.restoreFactoryDefaults();
        
        SparkHelper.setPIDGains(mAngle, 0, 0.25, 0, 0, 0);
        mAngle.setSmartCurrentLimit(30);

        mAngle.setInverted(angInverted);
        mAngle.setIdleMode(IdleMode.kBrake);
        resetToAbsolute();
    }

    private void configDriveMotor()
    {
        mDrive.restoreFactoryDefaults();
        
        SparkHelper.setPIDGains(mDrive, 0, TechConstants.kDriveVelocityKp, TechConstants.kDriveVelocityKi, TechConstants.kDriveVelocityKd, TechConstants.kDriveVelocityKf, TechConstants.kDriveVelocityRampRate, TechConstants.kDriveVelocityIZone);
        mDrive.setSmartCurrentLimit(30);
        mDrive.setOpenLoopRampRate(0.15);
    }

    public SwerveModuleState getState()
    {
        return new SwerveModuleState(mDrive.getEncoder().getVelocity() * TechConstants.kDriveRPM_To_MetersPerSec,
        getAngle());
    }

    public SwerveModulePosition getPosition()
    {
        return new SwerveModulePosition(
            mDrive.getEncoder().getPosition() * TechConstants.kRotationsToMetersConversionFactor,
            getAngle()
        );
    }
}
