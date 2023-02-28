package frc.robot.Autonomous.Framework;

import frc.robot.Utilities.Geometry.Pose2d;
import frc.robot.Actions.Framework.Action;
import frc.robot.Utilities.Geometry.Translation2d;
import frc.robot.Utilities.PathPlanner.PathPlannerTrajectory;
import frc.robot.Utilities.PathPlanner.PathPlannerTrajectory.PathPlannerState;
import frc.robot.Utilities.TrajectoryFollowing.Trajectory;
import frc.robot.Utilities.TrajectoryFollowing.Trajectory.State;

/**
 * An abstract class that is the basis of the robot's autonomous routines. This is implemented in auto modes (which are
 * routines that do actions).
 */
public abstract class AutoModeBase {
    protected double m_update_rate = 1.0 / 50.0;    //20ms update rate
    protected boolean m_active = false;

    protected abstract void routine() throws AutoModeEndedException;

    public void run() {
        m_active = true;
        try {
            routine();
        } catch (AutoModeEndedException e) {
            System.out.println("Auto mode done, ended early");
            return;
        }

        done();
        System.out.println("Auto mode done");
    }

    public void done() {
    }

    public void stop() {
        m_active = false;
    }

    public boolean isActive() {
        return m_active;
    }

    public boolean isActiveWithThrow() throws AutoModeEndedException {
        if (!isActive()) {
            throw new AutoModeEndedException();
        }

        return isActive();
    }

    public void runAction(Action action) throws AutoModeEndedException {
        isActiveWithThrow();
        action.start();

        while (isActiveWithThrow() && !action.isFinished()) {
            action.update();
            long waitTime = (long) (m_update_rate * 1000.0);

            try {
                Thread.sleep(waitTime);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }

        }

        action.done();
    }

    /**
     * 
     * Function to quickly grab the initial Pose2d from a given trajectory
     * 
     * (FOR HOLONOMIC DRIVETRAINS)
     * 
     * @param trajectory Trajectory from which to pull the initial expected position from
     * @return The pose sampled at time 0 from the trajectory
     */
    public Pose2d getStartingPose( PathPlannerTrajectory trajectory ) {
        PathPlannerState state = (PathPlannerState) trajectory.sample(0);
        return new Pose2d( new Translation2d( state.poseMeters.x(), state.poseMeters.y() ) , state.holonomicRotation );

    }

    /**
     * 
     * Function to quickly grab the initial Pose2d from a given trajectory
     * (FOR TANK DRIVETRAINS)
     * 
     * @param trajectory Trajectory from which to pull the initial expected position from
     * @return
     */
    public Pose2d getStartingPose( Trajectory trajectory ) {
        State state = trajectory.sample(0);
        return new Pose2d( new Translation2d( state.poseMeters.x(), state.poseMeters.y() ) , state.poseMeters.getRotation() );
    }

}