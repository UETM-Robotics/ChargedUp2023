package frc.robot.subsystems;

import com.revrobotics.CANSparkMax.ControlType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Loops.Loop;
import frc.robot.Loops.Looper;
import frc.robot.Utilities.Controllers;
import frc.robot.Utilities.CustomSubsystem;
import frc.robot.Utilities.Drivers.SparkHelper;
import frc.robot.Utilities.Drivers.SparkMaxU;

public class Lift extends Subsystem implements CustomSubsystem
{
    private static Lift instance = new Lift();

    SparkMaxU liftMotor;

    private boolean yay;

    public static Lift getInstance()
    {
        return instance;
    }

    private Lift()
    {
        liftMotor = Controllers.getInstance().getLiftMotor();

        //Try increasing kP?
        SparkHelper.setPIDGains(liftMotor, 0, 0.0001, 0, 0, 0);
        liftMotor.getPIDController().setOutputRange(-1, 1);
        
        yay = false;
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
            // TODO Auto-generated method stub
            if(yay)
            {
                SmartDashboard.putString("Success On", liftMotor.getPIDController().setReference(40, ControlType.kPosition).name());                
            }

            else
            {
                SmartDashboard.putString("Success Off", liftMotor.getPIDController().setReference(0, ControlType.kPosition).name());  
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

    public void doit()
    {
        yay = !yay;
    }

    /*public boolean pos()
    {
        //return liftMotor.get() >= 40;
    }*/
    
}
