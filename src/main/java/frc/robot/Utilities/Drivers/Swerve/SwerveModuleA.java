/*package frc.robot.Utilities.Drivers.Swerve;

import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Utilities.Constants.TechConstants;
import frc.robot.Utilities.Drivers.SparkMaxU;

public class SwerveModuleA
{
    private final SparkMaxU driver;
    private final SparkMaxU turning;

    private final CANCoder canCoder;

    private final PIDController turningPidController;
 
    private final boolean absEncReversed;
    private final double offsetRad;

    public SwerveModule(SparkMaxU driver, SparkMaxU turning, CANCoder canCoder, boolean driveMotorReversed,
                        double offsetRad, boolean turningMotorReversed, boolean absEncReversed)
    {
        this.driver = driver;
        this.turning = turning;

        driver.setOpenLoopRampRate(0.2);

        this.canCoder = canCoder;
        canCoder.configAbsoluteSensorRange(AbsoluteSensorRange.Unsigned_0_to_360);

        driver.setInverted(driveMotorReversed);
        turning.setInverted(turningMotorReversed);

        driver.getEncoder().setPositionConversionFactor(TechConstants.kDriveRotationsToMeters);
        driver.getEncoder().setVelocityConversionFactor(TechConstants.kDriveRPM_To_MetersPerSec);

        this.offsetRad = offsetRad;
        this.absEncReversed = absEncReversed;

        turningPidController = new PIDController(0.25, 0, 0);
        turningPidController.enableContinuousInput(-Math.PI, Math.PI);

        resetEncoders();
    }

    public double getDrivePos()
    {
        return driver.getEncoder().getPosition();
    }

    public double getTurningPos()
    {
        return turning.getEncoder().getPosition();
    }

    public double getDriveVel()
    {
        return driver.getEncoder().getVelocity();
    }

    public double getTurningVel()
    {
        return turning.getEncoder().getVelocity();
    }

    public double getAbsoluteEncoderRad()
    {
        double angle = canCoder.getAbsolutePosition() * Math.PI / 180 - offsetRad;

        return angle;
    }

    public void resetEncoders()
    {
        driver.getEncoder().setPosition(0);
        turning.getEncoder().setPosition(getAbsoluteEncoderRad());
    }

    public SwerveModuleState getState()
    {
        return new SwerveModuleState(getDriveVel(), new Rotation2d(getTurningPos()));
    }

    public void setDesiredState(SwerveModuleState state)
    {
        if(Math.abs(state.speedMetersPerSecond) < 0.001)
        {
            stop();
            return;
        }

        state = SwerveModuleState.optimize(state, getState().angle);
        driver.set(state.speedMetersPerSecond / TechConstants.kDriveMaxVelocity);
        turning.set(turningPidController.calculate(getTurningPos(), state.angle.getRadians()));

    }

    public void stop()
    {
        driver.set(0);
        turning.set(0);
    }
}
*/