package frc.robot.Actions;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotState;
import frc.robot.Actions.Framework.Action;
//import frc.robot.Loops.RobotStateEstimator;
import frc.robot.Utilities.DriveSignal;
import frc.robot.Utilities.ElapsedTimer;
import frc.robot.Utilities.Geometry.Pose2d;
import frc.robot.Utilities.TrajectoryFollowing.RamseteController;
import frc.robot.Utilities.TrajectoryFollowing.Trajectory;
import frc.robot.Utilities.TrajectoryFollowing.Trajectory.State;
import frc.robot.subsystems.DriveTrain;
import edu.wpi.first.wpilibj2.command.RamseteCommand;

/**
 * A command that uses a RAMSETE controller ({@link RamseteController}) to follow a trajectory
 * {@link Trajectory} with a differential drive.
 *
 * <p>The command handles trajectory-following, PID calculations, and feedforwards internally. This
 * is intended to be a more-or-less "complete solution" that can be used by teams without a great
 * deal of controls expertise.
 *
 * <p>Advanced teams seeking more flexibility (for example, those who wish to use the onboard PID
 * functionality of a "smart" motor controller) may use the secondary constructor that omits the PID
 * and feedforward functionality, returning only the raw wheel speeds from the RAMSETE controller.
 *
 * <p>This class is provided by the NewCommands VendorDep
 */
public class DriveTrajectoryAction implements Action{

  //private final Timer m_timer = new Timer();
  private final ElapsedTimer m_timer = new ElapsedTimer();

  private final Trajectory m_trajectory;

  //private final SimpleMotorFeedforward m_feedforward;

  //private final PIDController m_leftController;
  //private final PIDController m_rightController;
  private double m_prevTime;

  private final RobotState robotState = RobotState.getInstance();
  private final RamseteController m_follower = RamseteController.getInstance();

  private final DriveTrain drive_ = DriveTrain.getInstance();


  /**
   * Constructs a new RamseteCommand that, when executed, will follow the provided trajectory.
   * Performs no PID control and calculates no feedforwards; outputs are the raw wheel speeds from
   * the RAMSETE controller, and will need to be converted into a usable form by the user.
   *
   * @param trajectory The trajectory to follow.
   */
  public DriveTrajectoryAction(Trajectory trajectory, Pose2d startPose) {
    m_trajectory = trajectory;
    // drive_.subsystemHome();
    // RobotStateEstimator.getInstance().resetOdometry( startPose );

  }


  //TODO: TEST AT REFERENCE METHOD
  @Override
  public boolean isFinished() {
    //return m_follower.atReference();
    //return m_timer.hasElapsed(m_trajectory.getTotalTimeSeconds());

    return m_timer.hasElapsed() > m_trajectory.getTotalTimeSeconds();
    //return true;
  }

  @Override
  public void update() {
    
    double curTime = m_timer.hasElapsed();

    if (m_prevTime < 0) {
      drive_.updatePathVelocitySetpoint( DriveSignal.NEUTRAL );
      m_prevTime = curTime;
      return;
    }

    var targetWheelSpeeds = robotState.toWheelSpeeds(
      m_follower.calculate( robotState.getFieldToVehicleMeters(), m_trajectory.sample(curTime) )
    );

    var leftSpeedSetpoint = targetWheelSpeeds.leftMetersPerSecond;
    var rightSpeedSetpoint = targetWheelSpeeds.rightMetersPerSecond;

    

    drive_.updatePathVelocitySetpoint( new DriveSignal(leftSpeedSetpoint, rightSpeedSetpoint) );
    m_prevTime = curTime;

    SmartDashboard.putNumber("Current Time", m_timer.hasElapsed());

  }

  @Override
  public void done() {
    //m_timer.stop();
    

    drive_.updatePathVelocitySetpoint( DriveSignal.NEUTRAL );
  }

  @Override
  public void start() {

    m_prevTime = -1;
    State s = m_trajectory.sample(m_trajectory.getTotalTimeSeconds());

    
    SmartDashboard.putNumber("Total Trajectory Time", m_trajectory.getTotalTimeSeconds());

    //SmartDashboard.putNumber("expected ending x position", s.poseMeters.x());

    //SmartDashboard.putNumber("expected starting degrees", s.poseMeters.getRotation().getDegrees());
    // //SmartDashboard.putNumber("length of trajectory", m_trajectory.getTotalTimeSeconds());
    // s = m_trajectory.sample(1.4);
    // SmartDashboard.putNumber("expected mid length degrees", s.poseMeters.getRotation().getDegrees());

    //m_timer.reset();
    m_timer.start();

  }
}