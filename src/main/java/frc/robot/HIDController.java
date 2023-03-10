package frc.robot;

import java.util.ArrayList;
import java.util.function.BooleanSupplier;

import com.ctre.phoenix.ParamEnum;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Actions.Autonomy.AutomatedAction;
import frc.robot.Actions.Framework.TeleopActionRunner;
import frc.robot.Actions.OperatedActions.LiftAction;
import frc.robot.Actions.OperatedActions.SetActuatedAction;
import frc.robot.Actions.OperatedActions.SetIntakeAction;
import frc.robot.Actions.OperatedActions.SwapGamePiece;
import frc.robot.Loops.CrashTrackingRunnable;
import frc.robot.Utilities.Controllers;
import frc.robot.Utilities.ControlsConsumerU;
import frc.robot.Utilities.Constants.ControlsConstants;
import frc.robot.Utilities.Constants.TechConstants;
import frc.robot.Utilities.Drivers.rcinput.ControllerU;
import frc.robot.Utilities.Drivers.rcinput.ControllerU.Direction;
//import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Lift;
//import frc.robot.subsystems.DriveTrain.DriveControlState;
import frc.robot.subsystems.Intake.state;
import frc.robot.subsystems.Lift.Position;

public class HIDController {
    
    private static final HIDController instance = new HIDController();

    public synchronized static HIDController getInstance() {
        return instance;
    }

    private final Controllers controllers = Controllers.getInstance();

    private final ControllerU driverController;

    private final Object taskRunningLock_ = new Object();

    private boolean firstRun = true;


    private static final double HID_RATE_CONTROL = 0.020;

    private final Notifier mHIDNotifier;

    private final ArrayList<BooleanSupplier> mControlFunctions = new ArrayList<>();

    private final CrashTrackingRunnable mHIDRunnable = new CrashTrackingRunnable() {

        @Override
		public void runCrashTracked() {
			synchronized (taskRunningLock_) {
				if (firstRun) {
					Thread.currentThread().setName("DriveControls");
					Thread.currentThread().setPriority(TechConstants.kControllerThreadPriority);
					firstRun = false;
				}
				try {
					for (BooleanSupplier b : mControlFunctions) {
						b.getAsBoolean();   //TODO: Generate report of active functions
					}
				} catch (Exception ex) {
                    DriverStation.reportError("Fatal Error Running HID", true);
				} catch (Throwable t) {
                    DriverStation.reportError("Fatal Error Running HID", true);
				}
			}

			TeleopActionRunner.processActions();
		}
    };

    private HIDController() {
        mHIDNotifier = new Notifier(mHIDRunnable);
		driverController = controllers.getDriverController();
		registerControls();
    }

    public void start() {
		synchronized (taskRunningLock_) {
			mHIDNotifier.startPeriodic(HID_RATE_CONTROL);
			//Pneumatics.getInstance().setCompressor(true);
		}
	}

	public void stop() {
		synchronized (taskRunningLock_) {
			mHIDNotifier.stop();
			//DriveTrain.getInstance().setControlMode(DriveControlState.DISABLED);
		}
	}

	/**
     * Register a button press control: Face Buttons, Bumpers, Start, and Share Buttons
     *
     * @param joystick the target joystick
     * @param button the button that will activate the command
     * @param controlFunction the lambda function that will run the command
     */
	private void registerButtonPressControl( ControllerU joystick, int button, ControlsConsumerU controlFunction ) {
		mControlFunctions.add(() -> {
			if (joystick.getRisingEdgeButton(button)) {
				controlFunction.accept(joystick, button);
				return true;
			}
			return false;
		});
	}

	/**
     * Register a trigger press control
     *
     * @param joystick the target joystick
     * @param trigger the trigger that will activate the command
     * @param controlFunction the lambda function that will run the command
     */
	private void registerTriggerPressControl( ControllerU joystick, int trigger, ControlsConsumerU controlFunction ) {
		mControlFunctions.add( () -> {
			if( joystick.getTriggerPressed(trigger, TechConstants.kControllerTriggerThreshold)) {
				controlFunction.accept( joystick, trigger );

				return true;
			}

			return false;
		});
	}

	private void registerTriggerPressRawControl( ControllerU joystick, int trigger, ControlsConsumerU controlFunction ) {
		mControlFunctions.add( () -> {
			if( joystick.getTriggerPressedRaw(trigger, TechConstants.kControllerTriggerThreshold)) {
				controlFunction.accept( joystick, trigger );

				return true;
			}

			return false;
		});
	}

	private void registerDpadControl( ControllerU joystick, Direction POV, ControlsConsumerU controlFunction ) {
		mControlFunctions.add( () -> {
			if(joystick.getRisingEdgeDpad(POV)) {
				controlFunction.accept( joystick, 0);

				return true;
			}

			return false;
		} );
	}

	/**
     * Register a DpadUp press control
     *
     * @param joystick the target joystick
     * @param controlFunction the lambda function that will run the command
     */
	private void registerDpadUpControl(ControllerU joystick, ControlsConsumerU controlFunction) {
		mControlFunctions.add( () -> {
			if(joystick.getRisingUpDpad()) {
				controlFunction.accept(joystick, 0);
				return true;
			}
			return false;
		} );
	}

		/**
     * Register a DpadDown press control
     *
     * @param joystick the target joystick
     * @param controlFunction the lambda function that will run the command
     */
	private void registerDpadDownControl(ControllerU joystick, ControlsConsumerU controlFunction) {
		mControlFunctions.add( () -> {
			if(joystick.getRisingDownDpad()) {
				controlFunction.accept(joystick, 0);
				return true;
			}
			return false;
		} );
	}


    //Declare all controls here
    //DO NOT DECLARE DRIVER CONTROLS HERE
    private void registerControls()
	{
		//Intake
		registerButtonPressControl(driverController, 5, (j, b) -> {
			TeleopActionRunner.runAction(AutomatedAction.fromAction(
				new SetIntakeAction(state.INTAKE, () -> j.getRawButton(b)), 300));
		});

		registerButtonPressControl(driverController, 6, (j, b) -> {
			TeleopActionRunner.runAction(AutomatedAction.fromAction(
				new SetIntakeAction(state.EXTAKE, () -> j.getRawButton(b)), 300));
		});

		registerButtonPressControl(driverController, 1, (j, b) -> {
			TeleopActionRunner.runAction(AutomatedAction.fromAction(
				new SetActuatedAction(), 300));
		});

		registerButtonPressControl(driverController, 2, (j, b) -> {
			TeleopActionRunner.runAction(AutomatedAction.fromAction(
				new SwapGamePiece(), 300));
		});

		//Lift
		registerDpadControl(driverController, Direction.UP, (j, b) -> {
			TeleopActionRunner.runAction(AutomatedAction.fromAction(
				new LiftAction(Direction.UP), 300));
		});

		registerDpadControl(driverController, Direction.RIGHT, (j, b) -> {
			TeleopActionRunner.runAction(AutomatedAction.fromAction(
				new LiftAction(Direction.RIGHT), 300));
		});

		registerDpadControl(driverController, Direction.DOWN, (j, b) -> {
			TeleopActionRunner.runAction(AutomatedAction.fromAction(
				new LiftAction(Direction.DOWN), 300));
		});

		registerDpadControl(driverController, Direction.LEFT, (j, b) -> {
			TeleopActionRunner.runAction(AutomatedAction.fromAction(
				new LiftAction(Direction.LEFT), 300));
		});

		registerTriggerPressRawControl(driverController, 3, (j, b) -> {
			TeleopActionRunner.runAction(AutomatedAction.fromAction(
				new LiftAction(() -> j.getTriggerPressed(3, TechConstants.kControllerTriggerThreshold), true), 300));
		});

		registerTriggerPressRawControl(driverController, 2, (j, b) -> {
			TeleopActionRunner.runAction(AutomatedAction.fromAction(
				new LiftAction(() -> j.getTriggerPressed(2, TechConstants.kControllerTriggerThreshold), false), 300));
		});

		mControlFunctions.add( () -> {

			SmartDashboard.putNumber("Robot Position X", RobotState.getInstance().getFieldToVehicleInches().getPose().x() );
			SmartDashboard.putNumber("Robot Position Y", RobotState.getInstance().getFieldToVehicleInches().getPose().y() );
			SmartDashboard.putNumber("Robot Position Theta", RobotState.getInstance().getFieldToVehicleInches().getRotation().getDegrees());
			SmartDashboard.putNumber("FR", controllers.getCanCoderFR().getAbsolutePosition());
			SmartDashboard.putNumber("FL", controllers.getCanCoderFL().getAbsolutePosition());
			SmartDashboard.putNumber("BR", controllers.getCanCoderBR().getAbsolutePosition());
			SmartDashboard.putNumber("BR RAD", controllers.getCanCoderBR().getAbsolutePosition() * Math.PI / 180);
			SmartDashboard.putNumber("BL", controllers.getCanCoderBL().getAbsolutePosition());

			SmartDashboard.putNumber("SPARK FR", controllers.getRightFrontAngleMotor().getEncoder().getPosition());
			SmartDashboard.putNumber("SPARK FL", controllers.getLeftFrontAngleMotor().getEncoder().getPosition());
			SmartDashboard.putNumber("SPARK BR", controllers.getRightHindSwerveModuleU().getRot());
			SmartDashboard.putNumber("SPARK BL", controllers.getLeftHindAngleMotor().getEncoder().getPosition());

			/*SmartDashboard.putNumber("FR CALC", controllers.getRightFrontSwerveModuleU().calc());
			SmartDashboard.putNumber("FL CALC", controllers.getLeftFrontSwerveModuleU().calc());
			SmartDashboard.putNumber("BR CALC", controllers.getRightHindSwerveModuleU().calc());
			SmartDashboard.putNumber("BL CALC", controllers.getLeftHindSwerveModuleU().calc());*/

			SmartDashboard.putNumber("FR HIGHEST", controllers.getRightFrontSwerveModuleU().getHighest());
			SmartDashboard.putNumber("FL HIGHEST", controllers.getLeftFrontSwerveModuleU().getHighest());
			SmartDashboard.putNumber("BR HIGHEST", controllers.getRightHindSwerveModuleU().getHighest());
			SmartDashboard.putNumber("BL HIGHEST", controllers.getLeftHindSwerveModuleU().getHighest());

			SmartDashboard.putNumber("FR LOWEST", controllers.getRightFrontSwerveModuleU().getLowest());
			SmartDashboard.putNumber("FL LOWEST", controllers.getLeftFrontSwerveModuleU().getLowest());
			SmartDashboard.putNumber("BR LOWEST", controllers.getRightHindSwerveModuleU().getLowest());
			SmartDashboard.putNumber("BL LOWEST", controllers.getLeftHindSwerveModuleU().getLowest());

			SmartDashboard.putNumber("BR PID RAD AHHHH", controllers.getRightHindSwerveModuleU().getAbsoluteEncoderRad());
			return true;
		} );		
    }

}