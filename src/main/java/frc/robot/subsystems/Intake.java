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
    private boolean actuated;

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

        SparkHelper.setPIDGains(actuator, 0, 0.001, 0, 0, 0.05);

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
                        intakeMotor.set(isCube ? 0.65 : -0.65);
                        SmartDashboard.putString("State", "INTAKE");
                        break;
                
                    case EXTAKE:
                        intakeMotor.set(isCube ? -0.65 : 0.65);
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
                    if(curState == state.EXTAKE)
                    {
                        actuator.set(0.3, ControlType.kSmartMotion);
                    }

                    else
                    {
                        actuator.set(9, ControlType.kSmartMotion);
                    }
                }

                else
                {
                    actuator.set(0, ControlType.kSmartMotion);
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
