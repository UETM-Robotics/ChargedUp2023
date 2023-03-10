/*package frc.robot.Actions;

import java.util.function.Supplier;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Utilities.Constants.TechConstants;
import frc.robot.subsystems.SwerveBase;

public class SwerveCmd extends CommandBase{
    private final SwerveBase swerve;
    private final Supplier<Double> xspd, yspd, turningspd;

    public SwerveCmd(SwerveBase swerve, Supplier<Double> xspd, Supplier<Double> yspd, Supplier<Double> turningspd)
    {
        this.swerve = swerve;

        this.xspd = xspd;
        this.yspd = yspd;
        this.turningspd = turningspd;

        addRequirements(swerve);
    }

    @Override
    public void execute()
    {
        double x = xspd.get();
        double y = yspd.get();
        double turn = turningspd.get();

        x = Math.abs(x) > 0.01 ? x : 0;
        y = Math.abs(x) > 0.01 ? y : 0;
        turn = Math.abs(x) > 0.01 ? turn : 0;

        x *= TechConstants.kDriveMaxVelocity;
        y *= TechConstants.kDriveMaxVelocity;
        turn *= TechConstants.kDriveMaxVelocity;

        ChassisSpeeds chassisSpeeds;

        //field oriented
        chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(x, y, turn, swerve.geRotation2d());

        SwerveModuleState[] states = TechConstants.swerve_kinematics.toSwerveModuleStates(chassisSpeeds);

        swerve.setModuleStates(states);
    }
    
    @Override
    public void end(boolean interrupted)
    {
        swerve.stopModules();
    }

    @Override
    public boolean isFinished()
    {
        SmartDashboard.putString("A", "B");
        return false;
    }
}
*/