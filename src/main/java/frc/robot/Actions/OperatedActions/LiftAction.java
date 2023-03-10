package frc.robot.Actions.OperatedActions;
import java.util.function.Supplier;

import frc.robot.Actions.Framework.Action;
import frc.robot.Utilities.Drivers.rcinput.ControllerU.Direction;
import frc.robot.subsystems.Lift;
import frc.robot.subsystems.Lift.Position;

public class LiftAction implements Action
{
    private Lift lift;
    private Direction pov;

    private boolean isOpenLoop;
    private boolean isUp;

    Supplier<Boolean> mButtonGetterMethod;

    public LiftAction(Direction pov)
    {
        lift = Lift.getInstance();
        this.pov = pov;

        isOpenLoop = false;
        isUp = false;

        mButtonGetterMethod = null;
    }

    public LiftAction(Supplier<Boolean> mButtonGetterMethod, boolean isUp)
    {
        lift = Lift.getInstance();
        pov = null;

        isOpenLoop = true;
        this.isUp = isUp;

        this.mButtonGetterMethod = mButtonGetterMethod;
    }

    @Override
    public boolean isFinished() {
        // TODO Auto-generated method stub
        if(isOpenLoop)
        {
            return !mButtonGetterMethod.get();
        }
        
        return true;
    }

    @Override
    public void update() {
        // TODO Auto-generated method stub
    }

    @Override
    public void done() {
        // TODO Auto-generated method stub
        if(isOpenLoop)
        {
            lift.setPos(Position.IDLE);
        }
    }

    @Override
    public void start() {
        // TODO Auto-generated method stub
        if(!isOpenLoop)
        {
            switch(pov)
            {
                case UP:
                    lift.setPos(Position.HIGH);
                    break;
    
                case RIGHT:
                    lift.setPos(Position.MID);
                    break;
    
                case DOWN:
                    lift.setPos(Position.PICKUP);
                    break;
    
                case LEFT:
                    lift.setPos(Position.LOW);
                    break;
    
                default:
                    break;
            }
        }

        else
        {
            if(isUp)
            {
                lift.setPos(Position.OPEN_LOOP_UP);
            }

            else
            {
                lift.setPos(Position.OPEN_LOOP_DOWN);
            }
        }
    }
}
