package frc.robot.Actions.OperatedActions;
import frc.robot.Actions.Framework.Action;
import frc.robot.Utilities.Drivers.rcinput.ControllerU.Direction;
import frc.robot.subsystems.Lift;
import frc.robot.subsystems.Lift.Position;

public class LiftAction implements Action
{
    private Lift lift;
    private Direction pov;

    public LiftAction(Direction pov)
    {
        lift = Lift.getInstance();
        this.pov = pov;
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
        lift.stop();
    }

    @Override
    public void start() {
        // TODO Auto-generated method stub
        switch(pov)
        {
            case UP:
                lift.setPos(Position.HIGH);
                break;

            case RIGHT:
                lift.setPos(Position.MID);
                break;

            case DOWN:
                lift.setPos(Position.LOW);
                break;

            case LEFT:
                lift.setPos(Position.PICKUP);
                break;

            default:
                break;
        }
    }
}
