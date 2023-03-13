package frc.robot.Utilities.Drivers.Swerve;

import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.REVLibError;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Utilities.Controllers;
import frc.robot.Utilities.SynchronousPIDF;
import frc.robot.Utilities.Constants.TechConstants;
import frc.robot.Utilities.Drivers.SparkHelper;
import frc.robot.Utilities.Drivers.SparkMaxU;
import frc.robot.Utilities.Geometry.Rotation2d;
import frc.robot.Utilities.Swerve.SwerveModuleState;

public class Mk4iSwerveModule {

    private final SparkMaxU driveMotor;
    private final SparkMaxU turningMotor;

    //private final PIDController turningPidController;
    private final SynchronousPIDF turningPidController;

    private final CANCoder absoluteEncoder;
    private final double absoluteEncoderOffsetRad;
    private double highest = 0;
    private double lowest = 0;

    boolean isreversed;
    int sign;

    private IdleMode mIdleMode = IdleMode.COAST;


    public Mk4iSwerveModule(SparkMaxU driveMotor, SparkMaxU turningMotor, CANCoder angleEncoder, double absoluteEncoderOffset, boolean reversed) {

        driveMotor.restoreFactoryDefaults();
        turningMotor.restoreFactoryDefaults();

        this.absoluteEncoderOffsetRad = absoluteEncoderOffset;


        this.driveMotor = driveMotor;
        this.turningMotor = turningMotor;
        //this.turningMotor.setIdleMode(com.revrobotics.CANSparkMax.IdleMode.kBrake);

        this.absoluteEncoder = angleEncoder;
        this.absoluteEncoder.configAbsoluteSensorRange(AbsoluteSensorRange.Signed_PlusMinus180);

        //this.driveMotor.setInverted(absoluteEncoder.getAbsolutePosition() < 0);

        this.driveMotor.setSmartCurrentLimit(30);
        this.driveMotor.setOpenLoopRampRate(0.15);

        //SparkHelper.setPIDGains(this.driveMotor, 0, TechConstants.kDriveVelocityKp, TechConstants.kDriveVelocityKi, TechConstants.kDriveVelocityKd, TechConstants.kDriveVelocityKf, 0);

        SparkHelper.setPIDGains(this.driveMotor, 0, TechConstants.kDriveVelocityKp, TechConstants.kDriveVelocityKi, TechConstants.kDriveVelocityKd, TechConstants.kDriveVelocityKf, TechConstants.kDriveVelocityRampRate, TechConstants.kDriveVelocityIZone);
        
        // driveEncoder = driveMotor.getEncoder();
        // turningEncoder = turningMotor.getEncoder();

        // driveEncoder.setPositionConversionFactor(ModuleConstants.kDriveEncoderRot2Meter);
        // driveEncoder.setVelocityConversionFactor(ModuleConstants.kDriveEncoderRPM2MeterPerSec);
        // turningEncoder.setPositionConversionFactor(ModuleConstants.kTurningEncoderRot2Rad);
        // turningEncoder.setVelocityConversionFactor(ModuleConstants.kTurningEncoderRPM2RadPerSec);

        //turningPidController = new PIDController(ModuleConstants.kPTurning, 0, 0);
        turningPidController = new SynchronousPIDF(0.25, 0, 0.0);

        //turningPidController.enableContinuousInput(-Matph.PI, Math.PI);
        turningPidController.setContinuous(true);
        turningPidController.setInputRange(-Math.PI, Math.PI);

        //turningMotor.setIdleMode(com.revrobotics.CANSparkMax.IdleMode.kBrake);
        //turningMotor.setInverted(isreversed);

        //turningPidController.setDeadband(0.2);

        driveMotor.setInverted(true);

        /*double angle = absoluteEncoder.getAbsolutePosition() - absoluteEncoderOffset;

        if(angle > 90 || angle < -90)
        {
            driveMotor.setInverted(!driveMotor.getInverted());
        }

        if(angle > 180)
        {
            angle -= 180;
        }

        else if(angle < -180)
        {
            angle += 180;
        }

        angle *= 0.0569074715508;

        if(turningMotor.getEncoder().setPosition(angle) == REVLibError.kOk)
        {
            SmartDashboard.putBoolean("Set Pos", true);
        }
        
        else
        {
            SmartDashboard.putBoolean("Set Pos", false);
        }*/

        resetEncoders();
    }

    public void setRot(double rot)
    {
        turningMotor.getEncoder().setPosition(rot);
    }

    public double getRot()
    {
        return turningMotor.getEncoder().getPosition();
    }

    public void setHighest(double val)
    {
        if(val > highest)
        {
            highest = val;
        }
    }

    public void setLowest(double val)
    {
        if(val < lowest)
        {
            lowest = val;
        }
    }

    public double getHighest()
    {
        return highest;
    }

    public double getLowest()
    {
        return lowest;
    }


    public void setIdleMode(IdleMode idleMode) {
        if(mIdleMode != idleMode) {

            mIdleMode = idleMode;

            if(mIdleMode == IdleMode.BRAKE) {
                driveMotor.setIdleMode(com.revrobotics.CANSparkMax.IdleMode.kBrake);
                turningMotor.setIdleMode(com.revrobotics.CANSparkMax.IdleMode.kBrake);
            } else {
                driveMotor.setIdleMode(com.revrobotics.CANSparkMax.IdleMode.kCoast);
                turningMotor.setIdleMode(com.revrobotics.CANSparkMax.IdleMode.kCoast);
            }

        }
    }


    public double getDrivePosition() {
        return driveMotor.getEncoder().getPosition();
    }

    public double getDrivePosition(ConversionMode conversionMode) {
        switch(conversionMode) {
            case INCHES:
                return driveMotor.getEncoder().getPosition() * TechConstants.kDriveRotationsToInches;
            case METERS:
                return driveMotor.getEncoder().getPosition() * TechConstants.kDriveRotationsToMeters;
            default:
                return driveMotor.getEncoder().getPosition();

        }
    }


    public double getDriveVelocity() {
        return driveMotor.getEncoder().getVelocity();
    }

    public synchronized double getDriveVelocity(ConversionMode conversionMode) {

        switch(conversionMode) {
            case INCHES:
                return driveMotor.getEncoder().getVelocity() * TechConstants.kDriveRPM_To_InchesPerSec;
            case METERS:
                return driveMotor.getEncoder().getVelocity() * TechConstants.kDriveRPM_To_MetersPerSec;
            default:
                return driveMotor.getEncoder().getVelocity();

        }

    }


    public double getTurningVelocity() {
        return turningMotor.getEncoder().getVelocity();
    }


    public synchronized double getAbsoluteEncoderRad() {
        //double angle = absoluteEncoder.getVoltage() / RobotController.getVoltage5V();
        //double angle = (turningMotor.getEncoder().getPosition() % 21.60509554140127) / 21.60509554140127;
        double angle = absoluteEncoder.getAbsolutePosition() - absoluteEncoderOffsetRad;

        if(angle > 180)
        {
            angle -= 360;
        }

        else if(angle < -180)
        {
            angle += 360;
        }

        angle = -angle;

        //angle = angle / 360.0 * 21.60509554140127;

        /*angle *= 2.0 * Math.PI;// / 180;
        //angle -= absoluteEncoderOffsetRad * Math.PI / 180;

        if(angle > Math.PI) {
            angle -= 2.0 * Math.PI;
        }

        if(angle < -Math.PI)
        {
            angle += 2.0 * Math.PI;
        }

        // if(angle < 3.14) {
        //     angle = -(Math.PI - angle);
        // } else {
        //     angle -= Math.PI;
        // }*/

        angle *= Math.PI / 180.0;

        return angle;
    }


    public void resetEncoders() {
        driveMotor.resetEncoder();

        double pos = absoluteEncoder.getAbsolutePosition() - absoluteEncoderOffsetRad;

        if(pos > 180)
        {
            pos -= 360;
        }

        else if(pos < -180)
        {
            pos += 360;
        }

        pos =  -pos;

        pos = pos / 360.0 * 21.60509554140127;

        //angle *= 2.0 * Math.PI;

        turningMotor.getEncoder().setPosition(pos);
        //turningMotor.getEncoder().setPosition((absoluteEncoder.getAbsolutePosition() - absoluteEncoderOffsetRad) % 21.60509554140127 / 21.60509554140127);
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(getDriveVelocity(ConversionMode.METERS), Rotation2d.fromRadians(getAbsoluteEncoderRad()));
    }


    public void setDesiredState(SwerveModuleState state) {

        

        if (Math.abs(state.speedMetersPerSecond) < 0.001) {

            if(mIdleMode == IdleMode.BRAKE) {

                //state = SwerveModuleState.optimize(state, getState().angle);
                //state = SwerveModuleState.optimizeU(state, getState().angle);

                turningPidController.setSetpoint(state.angle.getRadians());

                //driveMotor.setVoltage(state.speedMetersPerSecond);
                driveMotor.set(0);
                turningMotor.set(turningPidController.calculate(getAbsoluteEncoderRad()));

            } else {

                stop();

            }
            
            return;
        }
        
        //state = SwerveModuleState.optimize(state, getState().angle);
        //state = SwerveModuleState.optimizeU(state, getState().angle);

        driveMotor.set(state.speedMetersPerSecond);

        turningPidController.setSetpoint(state.angle.getRadians());
        //turningPidController.setSetpoint(0);
        //SmartDashboard.putNumber("state mandated angle", state.);
        turningMotor.set(turningPidController.calculate(getAbsoluteEncoderRad()));
        

        //SmartDashboard.putString("Swerve[" + absoluteEncoder.getChannel() + "] state", state.toString());

    }

    public void setDesiredState(SwerveModuleState state, boolean usePID) {

        

        if (Math.abs(state.speedMetersPerSecond) < 0.001) {

            if(mIdleMode == IdleMode.BRAKE) {

                //state = SwerveModuleState.optimize(state, getState().angle);
                //state = SwerveModuleState.optimizeU(state, getState().angle);

                turningPidController.setSetpoint(state.angle.getRadians());

                //driveMotor.setVoltage(state.speedMetersPerSecond);
                driveMotor.set(0);
                turningMotor.set(turningPidController.calculate(getAbsoluteEncoderRad()));

            } else {

                stop();

            }
            
            return;
        }
        
        //state = SwerveModuleState.optimize(state, getState().angle);
        //state = SwerveModuleState.optimizeU(state, getState().angle);


        if(usePID) {
            driveMotor.set(state.speedMetersPerSecond, ControlType.kVelocity);
        } else {
            driveMotor.set(state.speedMetersPerSecond);
        }

        turningPidController.setSetpoint(state.angle.getRadians());
        //turningPidController.setSetpoint(0);
        //SmartDashboard.putNumber("state mandated angle", state.);
        turningMotor.set(turningPidController.calculate(getAbsoluteEncoderRad()));
        

        //SmartDashboard.putString("Swerve[" + absoluteEncoder.getChannel() + "] state", state.toString());

    }

    
    public void forceDesiredState(SwerveModuleState state) {
        
        //state = SwerveModuleState.optimize(state, getState().angle);
        //state = SwerveModuleState.optimizeU(state, getState().angle);


        driveMotor.set(state.speedMetersPerSecond);

        turningPidController.setSetpoint(state.angle.getRadians());
        //turningPidController.setSetpoint(0);
        //SmartDashboard.putNumber("state mandated angle", state.);
        turningMotor.set(turningPidController.calculate(getAbsoluteEncoderRad()));

    }

    public void stop() {

        driveMotor.set(0);
        turningMotor.set(0);

    }


    public enum ConversionMode {
        ROTATIONS,
        INCHES,
        METERS;
    }


    public enum IdleMode {
        BRAKE,
        COAST;
    }
}