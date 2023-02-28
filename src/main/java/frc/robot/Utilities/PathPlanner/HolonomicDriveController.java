// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Utilities.PathPlanner;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Utilities.SynchronousPIDF;
import frc.robot.Utilities.Constants.TechConstants;
import frc.robot.Utilities.Geometry.Pose2d;
import frc.robot.Utilities.Geometry.Rotation2d;
import frc.robot.Utilities.Swerve.ChassisSpeeds;
import frc.robot.Utilities.TrajectoryFollowing.Trajectory;

/**
 * This holonomic drive controller can be used to follow trajectories using a holonomic drivetrain
 * (i.e. swerve or mecanum). Holonomic trajectory following is a much simpler problem to solve
 * compared to skid-steer style drivetrains because it is possible to individually control forward,
 * sideways, and angular velocity.
 *
 * <p>The holonomic drive controller takes in one PID controller for each direction, forward and
 * sideways, and one profiled PID controller for the angular direction. Because the heading dynamics
 * are decoupled from translations, users can specify a custom heading that the drivetrain should
 * point toward. This heading reference is profiled for smoothness.
 */
@SuppressWarnings("MemberName")
public class  HolonomicDriveController {
    
  private Pose2d m_poseError = new Pose2d();
  private Rotation2d m_rotationError = new Rotation2d();
  private Pose2d m_poseTolerance = new Pose2d();
  private boolean m_enabled = true;

  private final SynchronousPIDF m_xController;
  private final SynchronousPIDF m_yController;
  private final SynchronousPIDF thetaControllerS;

  
  private final ProfiledPIDController m_thetaController;


  private boolean m_firstRun = true;

  /**
   * Constructs a holonomic drive controller.
   *
   * @param xController A PID Controller to respond to error in the field-relative x direction.
   * @param yController A PID Controller to respond to error in the field-relative y direction.
   * @param thetaController A profiled PID controller to respond to error in angle.
   */
  @SuppressWarnings("ParameterName")
  public HolonomicDriveController(
      SynchronousPIDF xController, SynchronousPIDF yController, ProfiledPIDController thetaController) {
    m_xController = xController;
    m_yController = yController;
    m_thetaController = thetaController;

    thetaControllerS = new SynchronousPIDF(TechConstants.kPThetaController, 0.0, 0.0);
    thetaControllerS.setContinuous(true);
    thetaControllerS.setInputRange(-Math.PI, Math.PI);
  }

  /**
   * Returns true if the pose error is within tolerance of the reference.
   *
   * @return True if the pose error is within tolerance of the reference.
   */
  public boolean atReference() {
    final var eTranslate = m_poseError.getTranslation();
    final var eRotate = m_rotationError;
    final var tolTranslate = m_poseTolerance.getTranslation();
    final var tolRotate = m_poseTolerance.getRotation();
    return Math.abs(eTranslate.x()) < tolTranslate.x()
        && Math.abs(eTranslate.y()) < tolTranslate.y()
        && Math.abs(eRotate.getRadians()) < tolRotate.getRadians();
  }

  /**
   * Sets the pose error which is considered tolerance for use with atReference().
   *
   * @param tolerance The pose error which is tolerable.
   */
  public void setTolerance(Pose2d tolerance) {
    m_poseTolerance = tolerance;
  }

  /**
   * Returns the next output of the holonomic drive controller.
   *
   * @param currentPose The current pose.
   * @param poseRef The desired pose.
   * @param linearVelocityRefMeters The linear velocity reference.
   * @param angleRef The angular reference.
   * @return The next output of the holonomic drive controller.
   */
  @SuppressWarnings("LocalVariableName")
  public ChassisSpeeds calculate(
      Pose2d currentPose, Pose2d poseRef, double linearVelocityRefMeters, Rotation2d angleRef) {
    // If this is the first run, then we need to reset the theta controller to the current pose's
    // heading.
    if (m_firstRun) {
      m_thetaController.reset(currentPose.getRotation().getRadians());
      m_firstRun = false;
    }

    // Calculate feedforward velocities (field-relative).
    double xFF = linearVelocityRefMeters * poseRef.getRotation().cos();
    double yFF = linearVelocityRefMeters * poseRef.getRotation().sin();

    double thetaFF =
        m_thetaController.calculate(-currentPose.getRotation().getRadians(), angleRef.getRadians());

    thetaControllerS.setSetpoint(angleRef.getRadians());
    thetaFF = thetaControllerS.calculate(currentPose.getRotation().getRadians());

    m_poseError = poseRef.relativeTo(currentPose);
    m_rotationError = angleRef.minus(currentPose.getRotation());

    if (!m_enabled) {
      return new ChassisSpeeds(xFF, yFF, thetaFF);
      //return ChassisSpeeds.fromFieldRelativeSpeeds(xFF, yFF, thetaFF, Rotation2d.fromDegrees(currentPose.getRotation().getDegrees()));
    }

    // Calculate feedback velocities (based on position error).
    m_xController.setSetpoint(poseRef.x());
    m_yController.setSetpoint(poseRef.y());

    double xFeedback = m_xController.calculate( currentPose.x() );
    double yFeedback = m_yController.calculate( currentPose.y() );

    SmartDashboard.putNumber("X Error", m_xController.getError());
    SmartDashboard.putNumber("Y Error", m_yController.getError());

    // Return next output.
    return new ChassisSpeeds(xFF + xFeedback, yFF + yFeedback, thetaFF);
    // return ChassisSpeeds.fromFieldRelativeSpeeds (
    //     xFF + xFeedback, yFF + yFeedback, thetaFF, Rotation2d.fromDegrees(currentPose.getRotation().getDegrees()) );

  }

  /**
   * Returns the next output of the holonomic drive controller.
   *
   * @param currentPose The current pose.
   * @param desiredState The desired trajectory state.
   * @param angleRef The desired end-angle.
   * @return The next output of the holonomic drive controller.
   */
  public ChassisSpeeds calculate(
      Pose2d currentPose, Trajectory.State desiredState, Rotation2d angleRef) {
    return calculate(
        currentPose, desiredState.poseMeters, desiredState.velocityMetersPerSecond, angleRef);
  }

  /**
   * Enables and disables the controller for troubleshooting problems. When calculate() is called on
   * a disabled controller, only feedforward values are returned.
   *
   * @param enabled If the controller is enabled or not.
   */
  public void setEnabled(boolean enabled) {
    m_enabled = enabled;
  }
}
