package frc.robot.Utilities.Drivers.Swerve;

import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.wpilibj.AnalogInput;

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

    private final CANCoder caNcoder;
    
    private final double absoluteEncoderOffsetRad;

    private IdleMode mIdleMode = IdleMode.COAST;


    public Mk4iSwerveModule(SparkMaxU driveMotor, SparkMaxU turningMotor, CANCoder caNcoder, double absoluteEncoderOffset) {

        driveMotor.restoreFactoryDefaults();
        turningMotor.restoreFactoryDefaults();

        this.absoluteEncoderOffsetRad = absoluteEncoderOffset;

        this.caNcoder = caNcoder;

        this.driveMotor = driveMotor;
        this.turningMotor = turningMotor;

        this.driveMotor.setInverted(true);


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

        //turningPidController.enableContinuousInput(-Math.PI, Math.PI);
        turningPidController.setContinuous(false);
        turningPidController.setInputRange(-Math.PI, Math.PI);

        resetEncoders();
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
        double angle = (turningMotor.getEncoder().getPosition() % 21.60509554140127) / 21.60509554140127;
        angle *= 2.0 * Math.PI;
        angle -= absoluteEncoderOffsetRad;

        if(angle > Math.PI) {
            angle -= 2.0 * Math.PI;
        }

        // if(angle < 3.14) {
        //     angle = -(Math.PI - angle);
        // } else {
        //     angle -= Math.PI;
        // }

        return angle;
    }


    public void resetEncoders() {
        driveMotor.resetEncoder();
        //turningEncoder.setPosition(getAbsoluteEncoderRad());
        turningMotor.setEncoderPosition(caNcoder.getAbsolutePosition());
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(getDriveVelocity(ConversionMode.METERS), Rotation2d.fromRadians(getAbsoluteEncoderRad()));
    }


    public void setDesiredState(SwerveModuleState state) {

        

        if (Math.abs(state.speedMetersPerSecond) < 0.001) {

            if(mIdleMode == IdleMode.BRAKE) {

                //state = SwerveModuleState.optimize(state, getState().angle);
                state = SwerveModuleState.optimizeU(state, getState().angle);

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
        state = SwerveModuleState.optimizeU(state, getState().angle);


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
                state = SwerveModuleState.optimizeU(state, getState().angle);

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
        state = SwerveModuleState.optimizeU(state, getState().angle);


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
        state = SwerveModuleState.optimizeU(state, getState().angle);


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