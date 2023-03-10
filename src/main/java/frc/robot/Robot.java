// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.CameraServerCvJNI;

//import java.util.Optional;

//import edu.wpi.first.cameraserver.CameraServer;
//import edu.wpi.first.cscore.MjpegServer;
//import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
//import frc.robot.Autonomous.Framework.AutoModeBase;
import frc.robot.Autonomous.Framework.AutoModeExecutor;
//import frc.robot.Autonomous.Modes.Test.Test;
import frc.robot.Loops.Looper;
import frc.robot.Loops.RobotStateEstimator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Lift;
import frc.robot.subsystems.SwerveDriveTrain;
import frc.robot.subsystems.SwerveDriveTrain.DriveControlState;
import frc.robot.Utilities.Controllers;
import frc.robot.Utilities.Constants.TechConstants;
//import frc.robot.Utilities.Controllers;
import frc.robot.Utilities.Geometry.Pose2d;
import frc.robot.Utilities.Geometry.Rotation2d;
//import frc.robot.Utilities.Swerve.SwerveModuleState;
import frc.robot.Utilities.Swerve.SwerveModuleState;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */

  private Looper mLooper;
  //private AutoModeSelector mAutoModeSelector = new AutoModeSelector();
  
  //private DriveTrain dTrain;
  private SwerveDriveTrain dTrain;
  private Intake intake;
  private Lift lift;
  private RobotStateEstimator robotStateEstimator;

  private AutoModeExecutor autoModeExecutor;
  private HIDController mHidController;
  
  //private MjpegServer server = new MjpegServer("Server", 0);

  @Override
  public void robotInit() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    CameraServer.startAutomaticCapture();

    mLooper = new Looper();

    dTrain = SwerveDriveTrain.getInstance();
    intake = Intake.getInstance();
    lift = Lift.getInstance();

    dTrain.init();
    dTrain.registerEnabledLoops(mLooper);    

    intake.registerEnabledLoops(mLooper);
    lift.registerEnabledLoops(mLooper);


    mHidController = HIDController.getInstance();

    robotStateEstimator = RobotStateEstimator.getInstance();
    mLooper.register(robotStateEstimator);

    autoModeExecutor = new AutoModeExecutor();

    // mAutoModeSelector.updateModeCreator();

  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();


    //mAutoModeSelector.outputToSmartDashboard();

    
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {

    exitAuto();

    mLooper.stop();

    // mAutoModeSelector.reset();
    // mAutoModeSelector.updateModeCreator();
    autoModeExecutor = new AutoModeExecutor();

    mHidController.stop();
    dTrain.setControlMode(DriveControlState.DISABLED);

  }

  @Override
  public void disabledPeriodic() {

    // mAutoModeSelector.updateModeCreator();
    // Optional<AutoModeBase> autoMode = mAutoModeSelector.getAutoMode();

    // if (autoMode.isPresent() && autoMode.get() != autoModeExecutor.getAutoMode()) {
    //   System.out.println("Set auto mode to: " + autoMode.get().getClass().toString());
    //   autoModeExecutor.setAutoMode(autoMode.get());
    // }

  }

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }

    
    /*autoModeExecutor.setAutoMode( new Test() );

    mLooper.start(true);
    autoModeExecutor.start();*/
    
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic()
  {
    
  }

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }


    try {
      System.out.println("Beginning TeleOP");

			exitAuto();

      //dTrain.setControlMode(DriveControlState.OPEN_LOOP);
      
			//dTrain.setBrakeMode(false);
      

      //TODO: ONLY FOR DEBUGGING
      robotStateEstimator.resetOdometry( new Pose2d(0, 0, Rotation2d.fromDegrees(0)) );

      
			mHidController.start();

      dTrain.setControlMode(DriveControlState.DRIVER_CONTROL);


      mLooper.start(true);


		} catch (Throwable t) {
      DriverStation.reportError("Fatal Error Initializing Teleop", true);
			throw t;
		}
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {

  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();

  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
  }
  
  private void exitAuto() {
		try {
			if (autoModeExecutor != null)
				autoModeExecutor.stop();


			autoModeExecutor = null;
		} catch (Throwable t) {
		}
	}

}
