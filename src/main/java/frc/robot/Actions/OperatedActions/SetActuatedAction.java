package frc.robot.Actions.OperatedActions;

import frc.robot.Actions.Framework.Action;
import frc.robot.subsystems.Intake;

public class SetActuatedAction implements Action
{
    private Intake intake;

    public SetActuatedAction()
    {
        intake = Intake.getInstance();
    }

    @Override
    public boolean isFinished() {
        return true;
    }

    @Override
    public void update() {
    }

    @Override
    public void done() {
    }

    @Override
    public void start() {
        intake.actuate();
    }
    
}
