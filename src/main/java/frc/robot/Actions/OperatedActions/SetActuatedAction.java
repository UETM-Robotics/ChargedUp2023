package frc.robot.Actions.OperatedActions;

import java.util.function.Supplier;

import frc.robot.Actions.Framework.Action;
import frc.robot.subsystems.Intake;

public class SetActuatedAction implements Action
{
    private Intake intake;
    private boolean pos;

    private Supplier<Boolean> mButtonGetterMethod;

    public SetActuatedAction(Supplier<Boolean> mButtonGetterMethod, boolean pos)
    {
        intake = Intake.getInstance();

        this.mButtonGetterMethod = mButtonGetterMethod;
        this.pos = pos;
    }

    @Override
    public boolean isFinished()
    {
        return !mButtonGetterMethod.get();
    }

    @Override
    public void update() {
    }

    @Override
    public void done()
    {
        intake.setVel(0);
    }

    @Override
    public void start()
    {
        intake.setVel(pos ? 0.2 : -0.2);
    }    
}