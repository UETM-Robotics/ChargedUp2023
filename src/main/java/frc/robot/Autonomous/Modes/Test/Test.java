package frc.robot.Autonomous.Modes.Test;



import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotState;
import frc.robot.Actions.SwerveDriveTrajectoryAction;
import frc.robot.Autonomous.Framework.AutoModeBase;
import frc.robot.Autonomous.Framework.AutoModeEndedException;
import frc.robot.Loops.RobotStateEstimator;
import frc.robot.Utilities.Geometry.Pose2d;
import frc.robot.Utilities.PathPlanner.PathPlanner;
import frc.robot.Utilities.PathPlanner.PathPlannerTrajectory;
import frc.robot.Utilities.TrajectoryFollowing.Trajectory;
import frc.robot.subsystems.SwerveDriveTrain;
import frc.robot.subsystems.SwerveDriveTrain.DriveControlState;
import frc.robot.subsystems.SwerveDriveTrain.DriveTrainBrakeMode;

public class Test extends AutoModeBase {

    PathPlannerTrajectory testPath = PathPlanner.loadPath("Perpendicular Driving", 1.0, 1.0);


    @Override
    protected void routine() throws AutoModeEndedException {


        SwerveDriveTrain.getInstance().setControlMode(DriveControlState.PATH_FOLLOWING);

        SwerveDriveTrain.getInstance().setBrakeMode(DriveTrainBrakeMode.RESIST_BRAKE);


        RobotStateEstimator.getInstance().resetOdometry( getStartingPose(testPath) );


        runAction( new SwerveDriveTrajectoryAction(testPath) );


        SwerveDriveTrain.getInstance().setControlMode(DriveControlState.DISABLED);

    }
    
}
