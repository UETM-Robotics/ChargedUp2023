package frc.robot.subsystems;

import com.revrobotics.CANEncoder;
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
    private double reference;

    public static Lift getInstance()
    {
        return instance;
    }

    private Lift()
    {
        liftMotor = Controllers.getInstance().getLiftMotor();

        //Try increasing kP?
        SparkHelper.setPIDGains(liftMotor, 0, 0.00014, 0, 0, 0.006);
        //SparkHelper.setSmartMotionParams(liftMotor, 0, 0, 0);
        liftMotor.getPIDController().setOutputRange(-1, 1);
        
        pos = Position.PICKUP;
        reference = 0;
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
                    reference = TechConstants.kPickup;
                    break;

                case LOW:
                    reference = TechConstants.kLow;
                    break;

                case MID:
                    reference = TechConstants.kMid;
                    break;

                case HIGH:
                    reference = TechConstants.kHigh;
                    break;

                default:
                    reference = TechConstants.kPickup;
                    SmartDashboard.putString("LIFT ERROR", "INVALID POS WTF HOW DID THIS EVEN HAPPEN");
                    break;
            }

            liftMotor.set(reference, ControlType.kSmartMotion);
        }

        @Override
        public void onStop(double timestamp) {
            // TODO Auto-generated method stub
            liftMotor.set(0);
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

    /*public boolean pos()
    {
        //return liftMotor.get() >= 40;
    }*/

    public enum Position
    {
        LOW,
        MID,
        HIGH,
        PICKUP
    }
    
}
