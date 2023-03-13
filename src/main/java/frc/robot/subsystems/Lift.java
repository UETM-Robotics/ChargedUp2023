package frc.robot.subsystems;

import com.revrobotics.CANSparkMax.ControlType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Loops.Loop;
import frc.robot.Loops.Looper;
import frc.robot.Utilities.Controllers;
import frc.robot.Utilities.CustomSubsystem;
import frc.robot.Utilities.Constants.TechConstants;
import frc.robot.Utilities.Drivers.SparkHelper;
import frc.robot.Utilities.Drivers.SparkMaxU;

public class Lift extends Subsystem implements CustomSubsystem
{
    private static Lift instance = new Lift();

    SparkMaxU liftMotor;

    private Position pos;

    public static Lift getInstance()
    {
        return instance;
    }

    private Lift()
    {
        liftMotor = Controllers.getInstance().getLiftMotor();

        SparkHelper.setPIDGains(liftMotor, 0, 0.00002, 0, 0, 0.003);
        //SparkHelper.setSmartMotionParams(liftMotor, 0, 0, 0);
        liftMotor.getPIDController().setOutputRange(-7, 4);
        
        pos = Position.PICKUP;

        prepare();
    }

    private final Loop mLoop = new Loop()
    {

        @Override
        public void onFirstStart(double timestamp) {
            // TODO Auto-generated method stub
        }

        @Override
        public void onStart(double timestamp) {
            // TODO Auto-generated method stub
        }

        @Override
        public void onLoop(double timestamp, boolean isAuto) 
        {
            switch(pos)
            {
                case PICKUP:
                    liftMotor.set(TechConstants.kPickup, ControlType.kSmartMotion);
                    break;

                case LOW:
                    liftMotor.set(TechConstants.kLow, ControlType.kSmartMotion);
                    break;

                case MID:
                    liftMotor.set(TechConstants.kMid, ControlType.kSmartMotion);
                    break;

                case HIGH:
                    liftMotor.set(-72, ControlType.kSmartMotion);
                    break;
                
                case OPEN_LOOP_UP:
                    liftMotor.set(-0.4);
                    break;

                case OPEN_LOOP_DOWN:
                    liftMotor.set(0.4);
                    break;
    
                case IDLE:
                    liftMotor.set(0);
                    break;

                default:
                    liftMotor.set(0);
                    SmartDashboard.putString("LIFT ERROR", "INVALID POS WTF HOW DID THIS EVEN HAPPEN");
                    break;
            }

            
        }

        @Override
        public void onStop(double timestamp) {
            // TODO Auto-generated method stub
        }
        
    };


    @Override
    public void init() {
        // TODO Auto-generated method stub
    }

    @Override
    public void subsystemHome() {
        // TODO Auto-generated method stub
    }

    @Override
    public void registerEnabledLoops(Looper in) {
        // TODO Auto-generated method stub
        in.register(mLoop);
    }

    @Override
    public void stop() {
        // TODO Auto-generated method stub
    }

    public void setPos(Position pos)
    {
        this.pos = pos;
    }

    public void prepare()
    {
        liftMotor.setSmartCurrentLimit(70);
        liftMotor.setOpenLoopRampRate(0.2);
    }

    public enum Position
    {
        LOW,
        MID,
        HIGH,
        PICKUP,
        OPEN_LOOP_UP,
        OPEN_LOOP_DOWN,
        IDLE
    }
    
}
