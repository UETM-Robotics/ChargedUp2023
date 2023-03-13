// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Utilities.Swerve;

import frc.robot.Utilities.Drivers.SparkMaxU;
import frc.robot.Utilities.Geometry.Rotation2d;
import java.util.Objects;

/** Represents the state of one swerve module. */
@SuppressWarnings("MemberName")
public class SwerveModuleState implements Comparable<SwerveModuleState> {
  /** Speed of the wheel of the module. */
  public double speedMetersPerSecond;

  /** Angle of the module. */
  public Rotation2d angle = Rotation2d.fromDegrees(0);

  /** Constructs a SwerveModuleState with zeros for speed and angle. */
  public SwerveModuleState() {}

  /**
   * Constructs a SwerveModuleState.
   *
   * @param speedMetersPerSecond The speed of the wheel of the module.
   * @param angle The angle of the module.
   */
  public SwerveModuleState(double speedMetersPerSecond, Rotation2d angle) {
    this.speedMetersPerSecond = speedMetersPerSecond;
    this.angle = angle;
  }

  @Override
  public boolean equals(Object obj) {
    if (obj instanceof SwerveModuleState) {
      return Double.compare(speedMetersPerSecond, ((SwerveModuleState) obj).speedMetersPerSecond)
          == 0;
    }
    return false;
  }

  @Override
  public int hashCode() {
    return Objects.hash(speedMetersPerSecond);
  }

  /**
   * Compares two swerve module states. One swerve module is "greater" than the other if its speed
   * is higher than the other.
   *
   * @param other The other swerve module.
   * @return 1 if this is greater, 0 if both are equal, -1 if other is greater.
   */
  @Override
  public int compareTo(SwerveModuleState other) {
    return Double.compare(this.speedMetersPerSecond, other.speedMetersPerSecond);
  }

  @Override
  public String toString() {
    return String.format(
        "SwerveModuleState(Speed: %.2f m/s, Angle: %s)", speedMetersPerSecond, angle);
  }



  //TODO: Further optimize angle control

  /**
   * Minimize the change in heading the desired swerve module state would require by potentially
   * reversing the direction the wheel spins. If this is used with the PIDController class's
   * continuous input functionality, the furthest a wheel will ever rotate is 90 degrees.
   *
   * @param desiredState The desired state.
   * @param currentAngle The current module angle.
   * @return Optimized swerve module state.
   */
  public static SwerveModuleState optimize(
      SwerveModuleState desiredState, Rotation2d currentAngle) {
    var delta = desiredState.angle.minus(currentAngle);
    if (Math.abs(delta.getDegrees()) > 90.0) {
      return new SwerveModuleState(
          -desiredState.speedMetersPerSecond,
          desiredState.angle.rotateBy(Rotation2d.fromDegrees(180.0)));
    } else {
      return new SwerveModuleState(desiredState.speedMetersPerSecond, desiredState.angle);
    }
  }

  public static SwerveModuleState optimizeU (
      SwerveModuleState desiredState, Rotation2d currentAngle) {


        // var delta = desiredState.angle.minus(currentAngle);


        //Rotation2d optionOne = desiredState.angle.plus(Rotation2d.fromRadians(Math.PI));  
        Rotation2d optionOne = Rotation2d.fromRadians( desiredState.angle.getRadians() + Math.PI );           //Rotate around and negate
        Rotation2d optionTwo = desiredState.angle;                                                            //Simply travel to the desired angle


        if(optionOne.getRadians() > Math.PI) {
          optionOne = Rotation2d.fromRadians( (optionOne.getRadians() - 2.0 * Math.PI) );
        }


        double optionOneError = 0;
        double optionTwoError = 0;


        optionOneError = optionOne.getRadians() - currentAngle.getRadians();
        optionTwoError = optionTwo.getRadians() - currentAngle.getRadians();


        if (Math.abs(optionOneError) > Math.PI) {

          if(optionOneError <= 0) {
            optionOneError = optionOneError + 2.0 * Math.PI;
          } else {
            optionOneError = optionOneError - 2.0 * Math.PI;
          }

        }


        if (Math.abs(optionTwoError) > Math.PI) {

          if(optionTwoError <= 0) {
            optionTwoError = optionTwoError + 2.0 * Math.PI;
          } else {
            optionTwoError = optionTwoError - 2.0 * Math.PI;
          }

        }


        if(Math.abs(optionOneError) < Math.abs(optionTwoError)) {
          return new SwerveModuleState(-desiredState.speedMetersPerSecond, optionOne);
        } else {
          return new SwerveModuleState(desiredState.speedMetersPerSecond, optionTwo);
        }


        // if (Math.abs(delta.getRadians()) > Math.PI) {
        //     if (delta.getRadians() > 0) {
        //         m_error = m_error - m_maximumInput + m_minimumInput;
        //     } else {
        //         m_error = m_error + m_maximumInput - m_minimumInput;
        //     }
        // }

  }
}
