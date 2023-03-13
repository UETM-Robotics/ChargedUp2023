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
    private final SparkMaxU actuator;
    private state curState = state.DEACTIVE;

    private double actuatedVel = 0;

    private boolean isCube = true;

    private static final Intake instance = new Intake();

    public static Intake getInstance()
    {
        return instance;
    }

    private Intake()
    {
        intakeMotor = Controllers.getInstance().getIntakeMotor();
        actuator = Controllers.getInstance().getActuatorMotor();

        SparkHelper.setPIDGains(actuator, 0, 0.000275, 0, 0, 0.005);
        actuator.getPIDController().setSmartMotionMaxVelocity(50, 0);
        actuator.getPIDController().setSmartMotionMaxAccel(50, 0);
        actuator.getPIDController().setOutputRange(-0.6, 0.4);

        actuator.setOpenLoopRampRate(0.1);
        actuator.setSmartCurrentLimit(30);

        intakeMotor.setOpenLoopRampRate(0.15);
        intakeMotor.setSmartCurrentLimit(40);
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
                        intakeMotor.set(0.75);
                        SmartDashboard.putString("State", "INTAKE");
                        break;
                
                    case EXTAKE:
                        intakeMotor.set(-0.75);
                        SmartDashboard.putString("State", "EXTAKE");
                        break;
                
                    default:
                        intakeMotor.set(0);
                        SmartDashboard.putString("ERROR", "INVALID INTAKE STATE THIS SHOULDN'T HAPPEN BUT IF YOU'RE SEEING THIS YOU REALLY SCREWED UP LMAO");
                        SmartDashboard.putString("State", "ERROR");
                        break;
                }

                actuator.set(-actuatedVel);

                SmartDashboard.putNumber("Motor Val", intakeMotor.get());
            }
        }

        @Override
        public void onStop(double timestamp) {
            // TODO Auto-generated method stub
            
        }
    };
    
    public void setVel(double output)
    {
        actuatedVel = output;
    }

    public void setState(state newState)
    {
        curState = newState;
    }

    public void swapGamePiece()
    {
        isCube = !isCube;
    }

    @Override
    public void init() {
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
