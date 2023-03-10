package frc.robot.Actions.OperatedActions;

import java.util.function.Supplier;

import frc.robot.Actions.Framework.Action;
import frc.robot.Utilities.Controllers;
import frc.robot.subsystems.Intake;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.Intake.state;

public class SetIntakeAction implements Action
{
    private Intake intake;
    private Intake.state state;

    private Supplier<Boolean> mButtonGetterMethod;

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
        Controllers.getInstance().getDriverController().setRumble(0);
    }

    @Override
    public void start() {
        // TODO Auto-generated method stub
        intake.setState(state);
        Controllers.getInstance().getDriverController().setRumble(0.4);
    }
    
}
