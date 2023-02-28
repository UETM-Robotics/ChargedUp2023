// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax.ControlType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Loops.Loop;
import frc.robot.Loops.Looper;
import frc.robot.Utilities.Controllers;
import frc.robot.Utilities.Drivers.SparkHelper;
import frc.robot.Utilities.Drivers.SparkMaxU;
import frc.robot.Utilities.CustomSubsystem;

public class Intake extends Subsystem implements CustomSubsystem {
    /** Creates a new ExampleSubsystem. */
    private final SparkMaxU intakeMotor;
    private final SparkMaxU acuator;
    private state curState = state.DEACTIVE;
    private boolean actuated;

    private static final Intake instance = new Intake();

    public static Intake getInstance()
    {
        return instance;
    }

    private Intake()
    {
        intakeMotor = Controllers.getInstance().getLiftMotor();
        acuator = Controllers.getInstance().getActuatorMotor();

        SparkHelper.setPIDGains(acuator, 0, 0.01, 0, 0, 0);

        actuated = false;
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
        public void onLoop(double timestamp, boolean isAuto) {
            synchronized (Intake.this) {

                switch(curState)
                {
                    case DEACTIVE:
                        intakeMotor.set(0);
                        SmartDashboard.putString("State", "DEACTIVE");
                        break;
                
                    case INTAKE:
                        intakeMotor.set(0.4);
                        SmartDashboard.putString("State", "INTAKE");
                        break;
                
                    case EXTAKE:
                        intakeMotor.set(-0.4);
                        SmartDashboard.putString("State", "EXTAKE");
                        break;
                
                    default:
                        intakeMotor.set(0);
                        SmartDashboard.putString("ERROR", "INVALID INTAKE STATE THIS SHOULDN'T HAPPEN BUT IF YOU'RE SEEING THIS YOU REALLY SCREWED UP LMAO");
                        SmartDashboard.putString("State", "ERROR");
                        break;
                }

                if(actuated)
                {
                    acuator.set(20, ControlType.kPosition);
                }

                else
                {
                    acuator.set(0, ControlType.kPosition);
                }

                SmartDashboard.putNumber("Motor Val", intakeMotor.get());
            }
        }

        @Override
        public void onStop(double timestamp) {
            // TODO Auto-generated method stub
            
        }
    };

    public void actuate()
    {
        actuated = !actuated;
    }

    public void setState(state newState)
    {
        if(curState != newState)
        {
            try
            {
                curState = newState;
            }
            
            catch (Exception e)
            {
                
            }
        }
    }

    public void test()
    {
        
    }

    public void prepareToEject() {
        intakeMotor.setSmartCurrentLimit(80);
        intakeMotor.setOpenLoopRampRate(0.2);
    }

    public void revert() {
        intakeMotor.setSmartCurrentLimit(50);
        intakeMotor.setOpenLoopRampRate(1);
    }


    @Override
    public void init() {
        intakeMotor.setSmartCurrentLimit(50);
        intakeMotor.setOpenLoopRampRate(1);
    }

    public enum state
    {
        DEACTIVE,
        INTAKE,
        EXTAKE
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
}
