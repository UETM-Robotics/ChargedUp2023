package frc.robot.Utilities;

import com.ctre.phoenix.sensors.CANCoder;
import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.SPI;
import frc.robot.Utilities.Constants.PortConstants;
import frc.robot.Utilities.Constants.TechConstants;
import frc.robot.Utilities.Drivers.CANSpeedControllerBuilder;
import frc.robot.Utilities.Drivers.SparkMaxU;
import frc.robot.Utilities.Drivers.Swerve.Mk4iSwerveModule;
import frc.robot.Utilities.Drivers.rcinput.ControllerU;
import frc.robot.Utilities.Geometry.Vector2d;

public class Controllers {

    private static final Controllers instance = new Controllers();

    private static final double TRACKWIDTH = TechConstants.kDriveTrackWidth;
    private static final double WHEELBASE = TechConstants.kDriveWheelBase;


    public static Controllers getInstance() {
        return instance;
    }

    private Controllers() {
        
        //leftFrontDriveMotor = CANSpeedControllerBuilder.createFastMasterSparkMax(PortConstants.leftFrontDriveMotorPort, 0);
        //leftHindDriveMotor = CANSpeedControllerBuilder.createPermanentSlaveSparkMax(PortConstants.leftHindDriveMotorPort, leftFrontDriveMotor);

        //rightFrontDriveMotor = CANSpeedControllerBuilder.createFastMasterSparkMax(PortConstants.rightFrontDriveMotorPort, 0);
        //rightHindDriveMotor = CANSpeedControllerBuilder.createPermanentSlaveSparkMax(PortConstants.rightHindDriveMotorPort, rightFrontDriveMotor);


        leftFrontThrottleMotor = CANSpeedControllerBuilder.createFastMasterSparkMax(PortConstants.leftFrontThrottleMotorPort, 0);
        leftFrontThrottleMotor.setInverted(false);
        leftFrontAngleMotor = CANSpeedControllerBuilder.createFastMasterSparkMax(PortConstants.leftFrontAngleMotorPort, 0);

        leftHindThrottleMotor = CANSpeedControllerBuilder.createFastMasterSparkMax(PortConstants.leftHindThrottleMotorPort, 0);
        leftHindThrottleMotor.setInverted(false);
        leftHindAngleMotor = CANSpeedControllerBuilder.createFastMasterSparkMax(PortConstants.leftHindAngleMotorPort, 0);

        rightFrontThrottleMotor = CANSpeedControllerBuilder.createFastMasterSparkMax(PortConstants.rightFrontThrottleMotorPort, 0);
        rightFrontThrottleMotor.setInverted(false);
        rightFrontAngleMotor = CANSpeedControllerBuilder.createFastMasterSparkMax(PortConstants.rightFrontAngleMotorPort, 0);

        rightHindThrottleMotor = CANSpeedControllerBuilder.createFastMasterSparkMax(PortConstants.rightHindThrottleMotorPort, 0);
        rightHindThrottleMotor.setInverted(false);
        rightHindAngleMotor = CANSpeedControllerBuilder.createFastMasterSparkMax(PortConstants.rightHindAngleMotorPort, 0);

        /*leftFrontThrottleMotor = CANSpeedControllerBuilder.createFastMasterSparkMax(3, 0);
        leftFrontAngleMotor = leftFrontThrottleMotor;
        leftHindThrottleMotor = leftFrontThrottleMotor;
        leftHindAngleMotor = leftFrontThrottleMotor;
        rightFrontThrottleMotor = leftFrontThrottleMotor;
        rightFrontAngleMotor = leftFrontThrottleMotor;
        rightHindThrottleMotor = leftFrontThrottleMotor;
        rightHindAngleMotor = leftFrontThrottleMotor;*/



        intakeMotor = CANSpeedControllerBuilder.createFastMasterSparkMax(PortConstants.intakeMotorPort, 0);
        liftMotor = CANSpeedControllerBuilder.createFastMasterSparkMax(PortConstants.liftMotorPort, 0);
        actuatorMotor = CANSpeedControllerBuilder.createFastMasterSparkMax(PortConstants.actuatorMotorPort, 0);

        liftMotor.setIdleMode(IdleMode.kBrake);
        actuatorMotor.setIdleMode(IdleMode.kBrake);

        CANCoder caNfr = new CANCoder(0);
        CANCoder caNfl = new CANCoder(0);
        CANCoder caNbr = new CANCoder(0);
        CANCoder caNbl = new CANCoder(0);

        gyro = new AHRS(SPI.Port.kMXP);


        //TODO: ADD IN ANGLE ENCODERS!!!

        // leftFrontModule = new Mk4iSwerveModule(
        //         new Vector2d(TRACKWIDTH / 2.0, WHEELBASE / 2.0),
        //         TechConstants.kFrontLeftAngleOffset,
        //         leftFrontAngleMotor,
        //         leftFrontThrottleMotor,
        //         new AnalogInput(0)
        // );

        // leftHindModule = new Mk4iSwerveModule(
        //     new Vector2d(-TRACKWIDTH / 2.0, WHEELBASE / 2.0),
        //     TechConstants.kHindLeftAngleOffset,
        //     leftHindAngleMotor,
        //     leftHindThrottleMotor,
        //     new AnalogInput(1)
        // );


        // rightFrontModule = new Mk4iSwerveModule(
        //     new Vector2d(TRACKWIDTH / 2.0, -WHEELBASE / 2.0),
        //     TechConstants.kFrontRightAngleOffset,
        //     rightFrontAngleMotor,
        //     rightFrontThrottleMotor,
        //     new AnalogInput(2)
        // );


        // rightHindModule = new Mk4iSwerveModule(
        //     new Vector2d(-TRACKWIDTH / 2.0, -WHEELBASE / 2.0),
        //     TechConstants.kHindRightAngleOffset,
        //     rightHindAngleMotor,
        //     rightHindThrottleMotor,
        //     new AnalogInput(3)
        // );


        leftFrontModuleU = new Mk4iSwerveModule(
            leftFrontThrottleMotor, 
            leftFrontAngleMotor, 
            caNfl, 
            0
        );

        rightFrontModuleU = new Mk4iSwerveModule(
            rightFrontThrottleMotor, 
            rightFrontAngleMotor, 
            caNfr, 
            0
        );

        leftHindModuleU = new Mk4iSwerveModule(
            leftHindThrottleMotor, 
            leftHindAngleMotor, 
           caNbl, 
            0
        );

        rightHindModuleU = new Mk4iSwerveModule(
            rightHindThrottleMotor, 
            rightHindAngleMotor,
            caNbr,
            0
        );

        driverController = new ControllerU(0);
    }

    private SparkMaxU leftFrontDriveMotor;
    private SparkMaxU leftHindDriveMotor;

    private SparkMaxU rightFrontDriveMotor;
    private SparkMaxU rightHindDriveMotor;


    private final SparkMaxU leftFrontThrottleMotor, leftFrontAngleMotor;

    private final SparkMaxU leftHindThrottleMotor, leftHindAngleMotor;

    private final SparkMaxU rightFrontThrottleMotor, rightFrontAngleMotor;

    private final SparkMaxU rightHindThrottleMotor, rightHindAngleMotor;

    private final SparkMaxU intakeMotor;//, actuatorMotor;
    private final SparkMaxU liftMotor;
    private final SparkMaxU actuatorMotor;


    private final Mk4iSwerveModule leftFrontModuleU, rightFrontModuleU;
    private final Mk4iSwerveModule leftHindModuleU , rightHindModuleU ;


    private final AHRS gyro;
    

    private final ControllerU driverController;

    public SparkMaxU getIntakeMotor()
    {
        return intakeMotor;
    }

    public SparkMaxU getLiftMotor()
    {
        return liftMotor;
    }

    public SparkMaxU getActuatorMotor()
    {
        return actuatorMotor;
    }

    public SparkMaxU getLeftFrontThrottleMotor() {
        return leftFrontThrottleMotor;
    }

    public SparkMaxU getRightFrontThrottleMotor() {
        return rightFrontThrottleMotor;
    }

    public SparkMaxU getLeftHindThrottleMotor() {
        return leftHindThrottleMotor;
    }

    public SparkMaxU getRightHindThrottleMotor() {
        return rightHindThrottleMotor;
    }

    
    public SparkMaxU getLeftFrontAngleMotor() {
        return leftFrontAngleMotor;
    }

    public SparkMaxU getRightFrontAngleMotor() {
        return rightFrontAngleMotor;
    }

    public SparkMaxU getLeftHindAngleMotor() {
        return leftHindAngleMotor;
    }

    public SparkMaxU getRightHindAngleMotor() {
        return rightHindAngleMotor;
    }


    public Mk4iSwerveModule getLeftFrontSwerveModuleU() {
        return leftFrontModuleU;
    }

    public Mk4iSwerveModule getRightFrontSwerveModuleU() {
        return rightFrontModuleU;
    }

    public Mk4iSwerveModule getLeftHindSwerveModuleU() {
        return leftHindModuleU;
    }

    public Mk4iSwerveModule getRightHindSwerveModuleU() {
        return rightHindModuleU;
    }

    
    


    public AHRS getGyro() {
        return gyro;
    }

    public ControllerU getDriverController() {
        return driverController;
    }

}
