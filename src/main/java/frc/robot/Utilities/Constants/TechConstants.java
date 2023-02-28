package frc.robot.Utilities.Constants;

import java.util.Dictionary;
import java.util.HashMap;
import java.util.Map;

import com.revrobotics.SparkMaxPIDController.AccelStrategy;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.robot.Utilities.SynchronousPIDF;
import frc.robot.Utilities.Geometry.Translation2d;
import frc.robot.Utilities.Swerve.SwerveDriveKinematics;

public class TechConstants {

    public static final double kLooperDt = 0.005;
    public static final int kLooperThreadPriority = 10;
    public static final int kControllerThreadPriority = 9;
    
    public static final double kJoystickJogThreshold = 0.4;
    public static final double kControllerTriggerThreshold = 0.75;
    public static final int kSparkMaxRetryCount = 3;
    public static final int kTimeoutMs = 20;
    public static final int kTimeoutMsFast = 10;

    public static final double kHubPositionXMeters = 8.7;
    public static final double kHubPositionYMeters = 4.1;
    public static final double kJoystickDeadband = 0.1;

    public static final double kRotationsToMetersConversionFactor = (0.1524 * Math.PI) / 10.71;
    public static final double kRPMtoMetersPerSecondConversionFactor = (0.1524 * Math.PI) / 60.0 / 10.71;


    public static final double kDriveTrackWidth = 0.573; //left middle of wheel to right middle of wheel
    public static final double kDriveWheelBase = 0.683; //front middle of wheel axle to back middle of wheel axle

    public static final double kFrontLeftAngleOffset = 0.0;
    public static final double kHindLeftAngleOffset = 0.0;
    public static final double kFrontRightAngleOffset = 0.0;
    public static final double kHindRightAngleOffset = 0.0;

    public static final double kDriveVelocityKp = 0.000003;
	public static final double kDriveVelocityKi = 0.8;//0.005;
	public static final double kDriveVelocityKd = 0;
	public static final double kDriveVelocityKf = 0.285;
	public static final double kDriveVelocityIZone = 0.05;
	public static final double kDriveVelocityRampRate = 0.1;
    public static final AccelStrategy kDriveAccelStrategy = AccelStrategy.kTrapezoidal;
    public static final double kDriveMaxVelocity = 4.0;
    public static final double kDriveMaxAccel = 2.0;

    //TODO: Figure these out
    public static final double kDriveRotationsToInches = 1.543780174;
    public static final double kDriveRotationsToMeters = 0.03921201641;
    public static final double kDriveRPM_To_InchesPerSec = 0.2094395103;
    public static final double kDriveRPM_To_MetersPerSec = 6.53533607E-4;


    //TODO: Figure these out
    public static final double kPickup = 0;
    public static final double kLow = 20;
    public static final double kMid = 40;
    public static final double kHigh = 60;


    public static final SwerveDriveKinematics swerve_kinematics = new SwerveDriveKinematics(

        new Translation2d(TechConstants.kDriveWheelBase / 2.0, -TechConstants.kDriveTrackWidth / 2.0),         // Front left
        new Translation2d(TechConstants.kDriveWheelBase / 2.0, TechConstants.kDriveTrackWidth / 2.0),        // Front right
        new Translation2d(-TechConstants.kDriveWheelBase / 2.0, -TechConstants.kDriveTrackWidth / 2.0),        // Back left
        new Translation2d(-TechConstants.kDriveWheelBase / 2.0, TechConstants.kDriveTrackWidth / 2.0)        // Back right
    );


    //TODO: TUNE WHEN ON CARPET
    public static final double kPXController = 0;
    public static final double kPYController = 0.1;
    public static final double kPThetaController = 0.4;

    public static final double kPhysicalMaxAngularSpeedRadiansPerSecond = 2 * 2 * Math.PI;
    public static final double kMaxAngularSpeedRadiansPerSecond = kPhysicalMaxAngularSpeedRadiansPerSecond / 10;
    public static final double kMaxAccelerationMetersPerSecondSquared = 3;
    public static final double kMaxAngularAccelerationRadiansPerSecondSquared = Math.PI / 4;

    public static final TrapezoidProfile.Constraints kThetaControllerConstraints = //
                new TrapezoidProfile.Constraints(
                        kMaxAngularSpeedRadiansPerSecond,
                        kMaxAngularAccelerationRadiansPerSecondSquared);


    public static final SynchronousPIDF xController = new SynchronousPIDF(kPXController, 0.0, 0.0);
    public static final SynchronousPIDF yController = new SynchronousPIDF(kPYController, 0.0, 0.0);
    public static final ProfiledPIDController thetaController = new ProfiledPIDController(
                                                                    kPThetaController, 
                                                                    0.0, 
                                                                    0.0,
                                                                    kThetaControllerConstraints
                                                                );
    
}
