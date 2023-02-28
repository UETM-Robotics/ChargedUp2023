package frc.robot.Actions;

import java.util.function.Consumer;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotState;
import frc.robot.Actions.Framework.Action;
import frc.robot.Utilities.ElapsedTimer;
import frc.robot.Utilities.HolonomicDriveSignal;
import frc.robot.Utilities.SynchronousPIDF;
import frc.robot.Utilities.Constants.TechConstants;
import frc.robot.Utilities.Geometry.Pose2d;
import frc.robot.Utilities.Geometry.Rotation2d;
import frc.robot.Utilities.Geometry.Vector2d;
import frc.robot.Utilities.PathPlanner.PathPlannerTrajectory;
import frc.robot.Utilities.PathPlanner.PathPlannerTrajectory.PathPlannerState;
import frc.robot.Utilities.Swerve.ChassisSpeeds;
import frc.robot.Utilities.Swerve.SwerveDriveKinematics;
import frc.robot.Utilities.Swerve.SwerveModuleState;
import frc.robot.subsystems.SwerveDriveTrain;
import frc.robot.subsystems.SwerveDriveTrain.DriveControlState;
import frc.robot.Utilities.PathPlanner.HolonomicDriveController;

/**
 * A command that uses two PID controllers ({@link PIDController}) and a
 * ProfiledPIDController
 * ({@link ProfiledPIDController}) to follow a trajectory {@link PathPlannerTrajectory}
 * with a swerve drive.
 *
 * <p>
 * This command outputs the raw desired Swerve Module States
 * ({@link SwerveModuleState}) in an
 * array. The desired wheel and module rotation velocities should be taken from
 * those and used in
 * velocity PIDs.
 *
 * <p>
 * The robot angle controller does not follow the angle given by the trajectory
 * but rather goes
 * to the angle given in the final state of the trajectory.
 *
 * <p>
 * This class is provided by the NewCommands VendorDep
 */
@SuppressWarnings("MemberName")
public class SwerveDriveTrajectoryAction implements Action {

    private final ElapsedTimer m_timer = new ElapsedTimer();
    private final PathPlannerTrajectory m_trajectory;

    
    private final HolonomicDriveController m_controller;

    private final RobotState robotState = RobotState.getInstance();

    private final SwerveDriveTrain driveTrain = SwerveDriveTrain.getInstance();


    private final boolean kShootWhileMoving;

    /**
     * Constructs a new PPSwerveControllerCommand that when executed will follow the
     * provided
     * trajectory. This command will not return output voltages but rather raw
     * module states from the
     * position controllers which need to be put into a velocity PID.
     *
     * <p>
     * Note: The controllers will *not* set the outputVolts to zero upon completion
     * of the path-
     * this is left to the user, since it is not appropriate for paths with
     * nonstationary endstates.
     *
     * @param trajectory         The trajectory to follow.
     */
    @SuppressWarnings("ParameterName")
    public SwerveDriveTrajectoryAction (
            PathPlannerTrajectory trajectory) {
                
        m_trajectory = trajectory;

        m_controller = new HolonomicDriveController(
                TechConstants.xController,
                TechConstants.yController,
                TechConstants.thetaController);

        m_controller.setEnabled(true);

        kShootWhileMoving = false;
    }

    /**
     * 
     * @param trajectory        The trajectory to follow
     * @param shootWhileMoving  **HIGHLY EXPERIMENTAL** set to true to enable shooting while moving
     */
    @SuppressWarnings("ParameterName")
    public SwerveDriveTrajectoryAction (
            PathPlannerTrajectory trajectory, boolean shootWhileMoving) {
                
        m_trajectory = trajectory;

        m_controller = new HolonomicDriveController(
                TechConstants.xController,
                TechConstants.yController,
                TechConstants.thetaController);

        m_controller.setEnabled(false);

        kShootWhileMoving = shootWhileMoving;
    }

    

    @Override
    public boolean isFinished() {
        return m_timer.hasElapsed() >= m_trajectory.getTotalTimeSeconds();
        //return m_timer.hasElapsed(m_trajectory.getTotalTimeSeconds());
    }

    @Override
    public void update() {

        double curTime = m_timer.hasElapsed();
        var desiredState = (PathPlannerState) m_trajectory.sample(curTime);

        ChassisSpeeds targetChassisSpeeds;

        //TODO: IMPLEMENT SHOOTING WHILE MOVING
        //**UNDER NORMAL CIRCUMSTANCES THIS SHOULD NOT RUN */
        //**DO NOT RUN THIS RIGHT NOW, THIS IS NOWHERE NEAR READY */
        if(kShootWhileMoving) {
            targetChassisSpeeds = m_controller.calculate(robotState.getFieldToVehicleMeters(),
                                                         desiredState,
                                                         desiredState.holonomicRotation);
        } else {
            targetChassisSpeeds = m_controller.calculate(robotState.getFieldToVehicleMeters(),
                                                         desiredState,
                                                         desiredState.holonomicRotation);
        }
        
        

        // HolonomicDriveSignal driveSignal = new HolonomicDriveSignal(
        //                                                             new Vector2d(targetChassisSpeeds.vyMetersPerSecond, targetChassisSpeeds.vxMetersPerSecond), 
        //                                                             targetChassisSpeeds.omegaRadiansPerSecond, 
        //                                                             true);
        

        HolonomicDriveSignal driveSignal = new HolonomicDriveSignal(
                                                                    new Vector2d(targetChassisSpeeds.vyMetersPerSecond, targetChassisSpeeds.vxMetersPerSecond), 
                                                                    targetChassisSpeeds.omegaRadiansPerSecond * 4.0 * Math.PI, 
                                                                    true);

        // HolonomicDriveSignal driveSignal = new HolonomicDriveSignal(
        //                                                             new Vector2d(0, 0), 
        //                                                             targetChassisSpeeds.omegaRadiansPerSecond * 4.0 * Math.PI, 
        //                                                             true);

        SmartDashboard.putNumber("drive signal x", driveSignal.getTranslation().x);
        SmartDashboard.putNumber("drive signal y", driveSignal.getTranslation().y);
        SmartDashboard.putNumber("drive signal theta", driveSignal.getRotation());

        SmartDashboard.putNumber("Desired Holonomic Rotation", desiredState.holonomicRotation.getDegrees());

        driveTrain.updateModules(driveSignal, true);

    }

    @Override
    public void done() {
        
        HolonomicDriveSignal driveSignal = new HolonomicDriveSignal(
                                                                    new Vector2d(0.0, 0.0), 
                                                                    0.0, 
                                                                    true);

        driveTrain.updateModules(driveSignal, false);

        System.out.println("Finished path!");
    }

    @Override
    public void start() {
        m_timer.start();
    }
}
