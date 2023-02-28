package frc.robot.Loops;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotState;
import frc.robot.Utilities.Constants.TechConstants;
import frc.robot.Utilities.Geometry.Pose2d;
import frc.robot.Utilities.Geometry.Rotation2d;
import frc.robot.Utilities.Geometry.Translation2d;
import frc.robot.Utilities.Swerve.SwerveDriveKinematics;
import frc.robot.Utilities.Swerve.SwerveDriveOdometry;
import frc.robot.Utilities.TrajectoryFollowing.DifferentialDriveOdometry;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.SwerveDriveTrain;

public class RobotStateEstimator implements Loop{

    private static final RobotStateEstimator instance = new RobotStateEstimator();

    SwerveDriveTrain drive_ = SwerveDriveTrain.getInstance();
    RobotState robotState = RobotState.getInstance();

    //private DifferentialDriveOdometry odometry;
    private SwerveDriveOdometry odometry;


    public static RobotStateEstimator getInstance() {
        return instance;
    }

    private RobotStateEstimator() {
        //odometry = new DifferentialDriveOdometry( drive_.getRotation() , new Pose2d(0, 0, Rotation2d.fromDegrees(0)));
        odometry = new SwerveDriveOdometry(TechConstants.swerve_kinematics, drive_.getRotation(), new Pose2d(0, 0, Rotation2d.fromDegrees(0)));
    }
    


    @Override
    public void onFirstStart(double timestamp) {
        
        
    }

    @Override
    public void onStart(double timestamp) {
        
        //odometry.resetPosition(new Pose2d(0, 0, Rotation2d.fromDegrees(0)), Rotation2d.fromDegrees(0));
    }

    @Override
    public void onLoop(double timestamp, boolean isAuto) {
        synchronized (RobotStateEstimator.this) {

            // robotState.updateFieldToRobotPose( odometry.update( drive_.getRotation(), 
            // drive_.getLeftDistanceMeters(), drive_.getRightDistanceMeters() ) );

            robotState.updateFieldToRobotPose(odometry.update(
                drive_.getRotation(),
                drive_.getModulestates()
            ));

        }
    }

    @Override
    public void onStop(double timestamp) {
        
        
    }

    public void resetOdometry() {
        drive_.subsystemHome();
        odometry.resetPosition(new Pose2d(0, 0, Rotation2d.fromDegrees(0)), drive_.getRotation());
    }

    public void resetOdometry(Pose2d startPose) {
        // drive_.subsystemHome();

        drive_.zeroGyro();
        odometry.resetPosition( startPose, drive_.getRotation() ) ;
        //odometry = new SwerveDriveOdometry(TechConstants.swerve_kinematics, drive_.getRotation(), startPose);
        //robotState.setInitialPose(startPose);
    }

}