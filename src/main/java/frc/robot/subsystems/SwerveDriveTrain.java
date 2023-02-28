package frc.robot.subsystems;

import java.util.concurrent.locks.ReentrantLock;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotState;
import frc.robot.Loops.Loop;
import frc.robot.Loops.Looper;
import frc.robot.Utilities.Controllers;
import frc.robot.Utilities.CustomSubsystem;
import frc.robot.Utilities.HolonomicDriveSignal;
import frc.robot.Utilities.SwerveKinematics;
import frc.robot.Utilities.SynchronousPIDF;
import frc.robot.Utilities.Constants.TechConstants;
import frc.robot.Utilities.Drivers.Swerve.Mk4iSwerveModule;
import frc.robot.Utilities.Drivers.Swerve.Mk4iSwerveModule.ConversionMode;
import frc.robot.Utilities.Drivers.Swerve.Mk4iSwerveModule.IdleMode;
import frc.robot.Utilities.Drivers.rcinput.ControllerU;
import frc.robot.Utilities.Geometry.Rotation2d;
import frc.robot.Utilities.Geometry.Translation2d;
import frc.robot.Utilities.Geometry.Vector2d;
import frc.robot.Utilities.Swerve.ChassisSpeeds;
import frc.robot.Utilities.Swerve.SwerveDriveKinematics;
import frc.robot.Utilities.Swerve.SwerveModuleState;

public class SwerveDriveTrain extends Subsystem implements CustomSubsystem {


    private static final SwerveDriveTrain instance = new SwerveDriveTrain();
    private final RobotState robotState = RobotState.getInstance();
    
    private DriveControlState mControlMode = DriveControlState.DISABLED;

    private final ReentrantLock _subsystemMutex = new ReentrantLock();

    private SwerveModuleState[] currModuleStates = new SwerveModuleState[4];
    private DriveTrainBrakeMode mBrakeMode;

    private ChassisSpeeds currDriveTrainVelocity;


    SynchronousPIDF thetaControllerS = new SynchronousPIDF(0.05, 0.0, 0.0);
    boolean firstNonRotatingRun = true;
    Rotation2d fieldRelativeOffset = Rotation2d.fromDegrees(0);

    double dt = 0;


    //TODO: Implement Brake Mode


    private SwerveDriveTrain() {

        leftFront = Controllers.getInstance().getLeftFrontSwerveModuleU();
        leftHind = Controllers.getInstance().getLeftHindSwerveModuleU();

        rightFront = Controllers.getInstance().getRightFrontSwerveModuleU();
        rightHind = Controllers.getInstance().getRightHindSwerveModuleU();

        modules = new Mk4iSwerveModule[] {leftFront, rightFront, leftHind, rightHind};

        driverController = Controllers.getInstance().getDriverController();

        gyro = Controllers.getInstance().getGyro();

        currDriveTrainVelocity = new ChassisSpeeds(0.0, 0.0, 0.0);


        thetaControllerS.setContinuous(true);
        thetaControllerS.setInputRange(-180, 180);


        new Thread(() -> {
            try {
                Thread.sleep(1000);
                gyro.reset();
                gyro.zeroYaw();
            } catch (Exception e) {
            }
        }).start();
    }

    public static SwerveDriveTrain getInstance() {
        return instance;
    }

    // private final Mk4iSwerveModule leftFront, rightFront;
    // private final Mk4iSwerveModule leftHind , rightHind;

    private final Mk4iSwerveModule leftFront, rightFront;
    private final Mk4iSwerveModule leftHind, rightHind;


    private final AHRS gyro;

    Mk4iSwerveModule[] modules;

    // private final SwerveDriveKinematics swerve_kinematics = new SwerveDriveKinematics(

    //     new Translation2d(TechConstants.kDriveTrackWidth / 2.0, TechConstants.kDriveWheelBase / 2.0),         // Front left
    //     new Translation2d(TechConstants.kDriveTrackWidth / 2.0, -TechConstants.kDriveWheelBase / 2.0),        // Front right
    //     new Translation2d(-TechConstants.kDriveTrackWidth / 2.0, TechConstants.kDriveWheelBase / 2.0),        // Back left
    //     new Translation2d(-TechConstants.kDriveTrackWidth / 2.0, -TechConstants.kDriveWheelBase / 2.0)        // Back right
    // );


    private final ControllerU driverController;

    private final PeriodicIO mPeriodicIO = new PeriodicIO();


    private final Loop mLoop = new Loop() {

        @Override
        public void onFirstStart(double timestamp) {
            
            gyro.zeroYaw();

        }

        @Override
        public void onStart(double timestamp) {
            
            fieldRelativeOffset = getRotation();

            gyro.zeroYaw();

            setBrakeMode(DriveTrainBrakeMode.BRAKE);

        }

        @Override
        public void onLoop(double timestamp, boolean isAuto) {
            synchronized (SwerveDriveTrain.this) {

                switch(mControlMode) {
                    
                    case DRIVER_CONTROL:

                        setBrakeMode(driverController.getRawButton(7) ? DriveTrainBrakeMode.BRAKE : DriveTrainBrakeMode.COAST );
                        //setBrakeMode(DriveTrainBrakeMode.BRAKE);

                        
                        mPeriodicIO.driveDemand = new HolonomicDriveSignal(driverController.getLeftJoystickVector().scale(0.3), 
                                                                           -driverController.getNormalizedAxis(4, TechConstants.kJoystickDeadband), true);
                                                                            
                        //mPeriodicIO.driveDemand = new HolonomicDriveSignal(new Vector2d(0, 0.5), 0.0, false);

                        updateModules(mPeriodicIO.driveDemand);
                        

                        break;
                    case OPEN_LOOP:
                        break;
                    case PATH_FOLLOWING:
                        break;
                    case TURN_TO_HEADING:
                        break;
                    case DISABLED:

                        for(Mk4iSwerveModule module : modules) {
                            module.setDesiredState( new SwerveModuleState(0.0, Rotation2d.fromDegrees(0)) );
                        }

                        break;
                    default:
                        break;

                }

                currDriveTrainVelocity = TechConstants.swerve_kinematics.toChassisSpeeds(currModuleStates);

                for(int i = 0; i < 4; i++) {
                    currModuleStates[i] = new SwerveModuleState(modules[i].getDriveVelocity(ConversionMode.METERS), Rotation2d.fromRadians(modules[i].getAbsoluteEncoderRad()));
                }
                
                SmartDashboard.putNumber("Robot Position X", robotState.getFieldToVehicleMeters().x());
                SmartDashboard.putNumber("Robot Position Y", robotState.getFieldToVehicleMeters().y());
                SmartDashboard.putNumber("Robot Position Theta", robotState.getFieldToVehicleMeters().getRotation().getDegrees());

                SmartDashboard.putNumber("Robot Velocity X", currDriveTrainVelocity.vxMetersPerSecond);
                SmartDashboard.putNumber("Robot Velocity Y", currDriveTrainVelocity.vyMetersPerSecond);
            }
        }

        @Override
        public void onStop(double timestamp) {
            stop();

            setBrakeMode(DriveTrainBrakeMode.COAST);
        }
        
    };


    @Override
    public void init() {
        
        currModuleStates[0] = new SwerveModuleState(0, Rotation2d.fromDegrees(0));
        currModuleStates[1] = new SwerveModuleState(0, Rotation2d.fromDegrees(0));
        currModuleStates[2] = new SwerveModuleState(0, Rotation2d.fromDegrees(0));
        currModuleStates[3] = new SwerveModuleState(0, Rotation2d.fromDegrees(0));
        
        
    }

    @Override
    public void subsystemHome() {
        
        new Thread(() -> {
            try {
                Thread.sleep(1000);
                gyro.reset();

                while(gyro.isCalibrating()) {

                }

                gyro.zeroYaw();

            } catch (Exception e) {
            }
        }).start();

        for(Mk4iSwerveModule module : modules) {
            module.resetEncoders();
        }
        
    }

    public void zeroGyro() {
        
        gyro.zeroYaw();
        
    }


    @Override
    public void registerEnabledLoops(Looper in) {
        in.register(mLoop);
    }

    @Override
    public void stop() {

        new Thread(() -> {
            try {
                Thread.sleep(1000);
                gyro.reset();
            } catch (Exception e) {
            }
        }).start();

        setBrakeMode(DriveTrainBrakeMode.COAST);
        
        //forceModules(new SwerveModuleState(0, Rotation2d.fromDegrees(0)) );
        
    }

    public synchronized void updateModules(HolonomicDriveSignal driveSignal) {

        ChassisSpeeds chassisSpeeds;

        driveSignal = new HolonomicDriveSignal(driveSignal.getTranslation().rotateBy(Rotation2d.fromDegrees(90)), 
                                               driveSignal.getRotation(), 
                                               driveSignal.isFieldOriented());


        if(driveSignal == null) {

            chassisSpeeds = new ChassisSpeeds(0, 0, 0);

        } else if(driveSignal.isFieldOriented()) {
            chassisSpeeds = new ChassisSpeeds(0, 0, 0);
            chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                                          driveSignal.getTranslation().x, 
                                          driveSignal.getTranslation().y, 
                                          driveSignal.getRotation(), 
                                          Rotation2d.fromDegrees(-RobotState.getInstance().getFieldToVehicleMeters().getRotation().getDegrees())
                            );

        } else {

            chassisSpeeds = new ChassisSpeeds(driveSignal.getTranslation().x, 
                                              driveSignal.getTranslation().y, 
                                              driveSignal.getRotation()
                                );
        }


        SwerveModuleState[] states = new SwerveModuleState[4];

        if(mBrakeMode == DriveTrainBrakeMode.RESIST_BRAKE && Math.abs(chassisSpeeds.movementVector.length) < 0.001 && Math.abs(chassisSpeeds.omegaRadiansPerSecond) < 0.001) {

            states[0] = new SwerveModuleState(0.0001, Rotation2d.fromDegrees(135) );
            states[1] = new SwerveModuleState(0.0001, Rotation2d.fromDegrees(-135) );
            states[2] = new SwerveModuleState(0.0001, Rotation2d.fromDegrees(45) );
            states[3] = new SwerveModuleState(0.0001, Rotation2d.fromDegrees(-45) );

        } else {
            states = TechConstants.swerve_kinematics.toSwerveModuleStates(chassisSpeeds);
            SwerveDriveKinematics.desaturateWheelSpeeds(states, 1);
        }


        

        for(int i = 0; i < states.length; i++) {
            
            modules[i].setDesiredState(states[i]);

        }

    }


    public synchronized void updateModules(HolonomicDriveSignal driveSignal, boolean usePID) {

        ChassisSpeeds chassisSpeeds;

        driveSignal = new HolonomicDriveSignal(driveSignal.getTranslation().rotateBy(Rotation2d.fromDegrees(90)), 
                                               driveSignal.getRotation(), 
                                               driveSignal.isFieldOriented());

        if(driveSignal == null) {

            chassisSpeeds = new ChassisSpeeds(0, 0, 0);

        } else if(driveSignal.isFieldOriented()) {
            chassisSpeeds = new ChassisSpeeds(0, 0, 0);
            chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                                          driveSignal.getTranslation().x, 
                                          driveSignal.getTranslation().y, 
                                          driveSignal.getRotation(), 
                                          Rotation2d.fromDegrees(-RobotState.getInstance().getFieldToVehicleMeters().getRotation().getDegrees())
                            );

        } else {

            chassisSpeeds = new ChassisSpeeds(driveSignal.getTranslation().x, 
                                              driveSignal.getTranslation().y, 
                                              driveSignal.getRotation()
                                );
        }


        SwerveModuleState[] states = new SwerveModuleState[4];

        if(mBrakeMode == DriveTrainBrakeMode.RESIST_BRAKE && Math.abs(chassisSpeeds.movementVector.length) < 0.001 && Math.abs(chassisSpeeds.omegaRadiansPerSecond) < 0.001) {

            states[0] = new SwerveModuleState(0.0001, Rotation2d.fromDegrees(135) );
            states[1] = new SwerveModuleState(0.0001, Rotation2d.fromDegrees(-135) );
            states[2] = new SwerveModuleState(0.0001, Rotation2d.fromDegrees(45) );
            states[3] = new SwerveModuleState(0.0001, Rotation2d.fromDegrees(-45) );

        } else {
            states = TechConstants.swerve_kinematics.toSwerveModuleStates(chassisSpeeds);
            SwerveDriveKinematics.desaturateWheelSpeeds(states, 3.5);
        }

        if(!usePID) {
            for(int i = 0; i < states.length; i++) {
            
                states[i].speedMetersPerSecond /= 3.5;

                modules[i].setDesiredState(states[i], false);
    
            }
        } else {
            for(int i = 0; i < states.length; i++) {
                modules[i].setDesiredState(states[i], true);
            }
        }

        

    }

    private synchronized void forceModules(SwerveModuleState state) {

        for(Mk4iSwerveModule module : modules) {
            module.forceDesiredState(state);
        }

    }


    public synchronized void setControlMode(DriveControlState controlMode) {
		if (controlMode != mControlMode) {
			try {
				_subsystemMutex.lock();
				mControlMode = controlMode;
				_subsystemMutex.unlock();
			} catch (Exception ex) {
                
			}
		}
	}

    public synchronized void setBrakeMode(DriveTrainBrakeMode brakeMode) {
		if (mBrakeMode != brakeMode) {
			try {
				_subsystemMutex.lock();
				mBrakeMode = brakeMode;
				_subsystemMutex.unlock();
			} catch (Exception ex) {
                
			}

            if(mBrakeMode == DriveTrainBrakeMode.RESIST_BRAKE || mBrakeMode == DriveTrainBrakeMode.RESIST_BRAKE) {
                for(Mk4iSwerveModule module : modules) {
                    module.setIdleMode(IdleMode.BRAKE);
                }
            } else {
                for(Mk4iSwerveModule module : modules) {
                    module.setIdleMode(IdleMode.COAST);
                }
            }
		}
	}


    public synchronized Rotation2d getRotation() {
        //return mNavXBoard.getYaw();
        return Rotation2d.fromDegrees(-gyro.getAngle() - fieldRelativeOffset.getDegrees());
    }

    public synchronized SwerveModuleState[] getModulestates() {
        return currModuleStates;
    }

    public synchronized ChassisSpeeds getCurrRobotChassisSpeeds() {
        return currDriveTrainVelocity;
    }

    @SuppressWarnings("WeakerAccess")
	public static class PeriodicIO {
        
        public HolonomicDriveSignal driveDemand = HolonomicDriveSignal.NEUTRAL;

	}
    
    public enum DriveControlState {
        OPEN_LOOP,
        PATH_FOLLOWING,
        DRIVER_CONTROL,
        TURN_TO_HEADING,
        DISABLED;
    }

    public enum DriveTrainBrakeMode {
        COAST,
        RESIST_BRAKE,
        BRAKE;
    }

}
