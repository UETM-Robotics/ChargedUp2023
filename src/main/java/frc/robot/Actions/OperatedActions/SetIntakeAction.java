package frc.robot.Actions.OperatedActions;

import java.util.function.Supplier;

import frc.robot.Actions.Framework.Action;
import frc.robot.subsystems.Intake;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SetIntakeAction implements Action
{
    Intake intake;
    Intake.state state;

    Supplier<Boolean> mButtonGetterMethod;

    public SetIntakeAction(Intake.state state, Supplier<Boolean> buttonGetterMethod)
    {
        intake = Intake.getInstance();
        this.state = state;
        mButtonGetterMethod = buttonGetterMethod;
    }

    @Override
    public boolean isFinished()
    {
        return !mButtonGetterMethod.get();
    }

    @Override
    public void update() {
        // TODO Auto-generated method stub
        
    }

    @Override
    public void done() {
        // TODO Auto-generated method stub
        intake.setState(state.DEACTIVE);
    }

    @Override
    public void start() {
        // TODO Auto-generated method stub
        intake.setState(state);
    }
    
}
