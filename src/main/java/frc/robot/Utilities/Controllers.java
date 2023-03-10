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
//import frc.robot.Utilities.Drivers.Swerve.SwerveModuleA;
import frc.robot.Utilities.Drivers.rcinput.ControllerU;
import frc.robot.Utilities.Geometry.Vector2d;
import frc.robot.subsystems.SwerveModule;

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

        caNfr = new CANCoder(PortConstants.canFr);
        caNfl = new CANCoder(PortConstants.canFl);
        caNbr = new CANCoder(PortConstants.canBr);
        caNbl = new CANCoder(PortConstants.canBl);

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
            190.01953125,
            true
        );

        rightFrontModuleU = new Mk4iSwerveModule(
            rightFrontThrottleMotor, 
            rightFrontAngleMotor, 
            caNfr, 
            203.73046875,
            true
        );

        leftHindModuleU = new Mk4iSwerveModule(
            leftHindThrottleMotor, 
            leftHindAngleMotor, 
            caNbl,
            60.46875,
            false
        );

        rightHindModuleU = new Mk4iSwerveModule(
            rightHindThrottleMotor, 
            rightHindAngleMotor,
            caNbr,
            228.69140625,
            false
        );

        /*rightFrontModuleU = new SwerveModule(
            rightFrontThrottleMotor,
            rightFrontAngleMotor, 
            caNfr,
            false,
            0,
            false,
            false
        );

        leftFrontModuleU = new SwerveModule(
            leftFrontThrottleMotor,
            actuatorMotor,
            caNfl,
            false,
            0,
            false,
            false
        );

        rightHindModuleU = new SwerveModule(
            rightHindThrottleMotor,
            rightHindAngleMotor, 
            caNbr,
            false,
            0,
            false,
            false
        );

        leftHindModuleU = new SwerveModule(
            leftHindThrottleMotor,
            leftHindAngleMotor,
            caNbl,
            false,
            0,
            false,
            false
        );*/

        /*leftFrontModule = new SwerveModule(
            leftFrontDriveMotor,
            leftFrontAngleMotor,
            caNfl,
            false,
            0,
            false,
            false
        );

        rightFrontModule = new SwerveModule(
            rightFrontDriveMotor,
            rightFrontAngleMotor,
            caNfr,
            false,
            0,
            false,
            false
        );

        leftBackModule = new SwerveModule(
            leftHindDriveMotor,
            leftHindAngleMotor,
            caNbl,
            false,
            0,
            false,
            false
        );

        rightBackModule = new SwerveModule(
            right
        );*/

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

    private final CANCoder caNfr;
    private final CANCoder caNfl;
    private final CANCoder caNbr;
    private final CANCoder caNbl;


    private final Mk4iSwerveModule leftFrontModuleU, rightFrontModuleU;
    private final Mk4iSwerveModule leftHindModuleU , rightHindModuleU;

    //private final SwerveModule leftFrontModule, rightFrontModule, leftBackModule, rightBackModule;


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

    
    /*public SwerveModule getRightFrontModule()
    {
        return rightFrontModule;
    }

    public SwerveModule getLeftFrontModule()
    {
        return leftFrontModule;
    }

    public SwerveModule getRightBackModule()
    {
        return rightBackModule;
    }

    public SwerveModule getLeftBackModule()
    {
        return leftBackModule;
    }*/


    public AHRS getGyro() {
        return gyro;
    }

    public ControllerU getDriverController() {
        return driverController;
    }

    public CANCoder getCanCoderFR()
    {
        return caNfr;
    }

    public CANCoder getCanCoderFL()
    {
        return caNfl;
    }

    public CANCoder getCanCoderBR()
    {
        return caNbr;
    }

    public CANCoder getCanCoderBL()
    {
        return caNbl;
    }
}
