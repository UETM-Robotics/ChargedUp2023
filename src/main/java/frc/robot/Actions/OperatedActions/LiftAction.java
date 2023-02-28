package frc.robot.Actions.OperatedActions;
import frc.robot.Actions.Framework.Action;
import frc.robot.subsystems.Lift;

public class LiftAction implements Action
{
    private Lift lift;

    public LiftAction()
    {
        lift = Lift.getInstance();
    }

    @Override
    public boolean isFinished() {
        // TODO Auto-generated method stub
        return true;
    }

    @Override
    public void update() {
        // TODO Auto-generated method stub
    }

    @Override
    public void done() {
        // TODO Auto-generated method stub
    }

    @Override
    public void start() {
        // TODO Auto-generated method stub
        lift.doit();
    }
}
