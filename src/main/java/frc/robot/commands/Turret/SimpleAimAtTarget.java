package frc.robot.commands.Turret;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

public class SimpleAimAtTarget extends Command{
    SwerveSubsystem swerveSubsystem;
    Turret turretSubsystem;
    Vision visionSubsystem;
    Pose2d target;
    
    public SimpleAimAtTarget (SwerveSubsystem swerve, Vision vision, Turret turret, Pose2d m_target){
        swerveSubsystem = swerve;
        visionSubsystem = vision;
        turretSubsystem = turret;
        target = m_target;
    }

    @Override
    public void initialize(){
        Pose2d swervePose = swerveSubsystem.getSwerveDrive().getPose();
        // may need to be updated depending on how we actually implement ZeroSwitchPos
        Pose2d turretPose = swervePose.rotateBy(Rotation2d.fromDegrees(Constants.ScoringConstants.ZeroSwitchPos + turretSubsystem.getTurretAngle()));
        Transform2d diff = target.minus(turretPose);
        double angleNeeded = Math.toDegrees(Math.atan2(diff.getY(), diff.getX()));
        // may also need to be updated
        turretSubsystem.setTurretAngle(Rotation2d.fromDegrees(angleNeeded - Constants.ScoringConstants.ZeroSwitchPos));
    }

    @Override
    public boolean isFinished(){
        return true;
    }
}
