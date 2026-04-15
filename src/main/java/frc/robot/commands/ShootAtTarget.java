// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.Arrays;
import java.util.List;
import java.util.Map;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.InverseInterpolator;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Flywheel;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Kicker;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.Constants.ShooterLookupTables;

/** An example command that uses an example subsystem. */
public class ShootAtTarget extends Command {
  @SuppressWarnings("PMD.UnusedPrivateField")

  

  private final Turret turret;
  private final Kicker kicker;
  private final Hood hood;
  private final SwerveSubsystem drivetrain;
  private final Flywheel flywheel;


  Rotation2d desiredTurretAngle = null;


  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
   public ShootAtTarget(Turret turret, Kicker kicker, Hood hood, SwerveSubsystem drivetrain, Flywheel flywheel) {
    this.kicker = kicker;
    this.turret = turret;
    this.hood = hood;
    this.drivetrain = drivetrain;
    this.flywheel = flywheel;

    addRequirements(turret);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Translation2d target = null;
    // m_turret.
    if (DriverStation.getAlliance().get() == Alliance.Blue){
      // hub = new Translation2d(39.05, 13.19);
      // target = new Translation2d(4.62, 4.03);
      if(drivetrain.getSwerveDrive().getPose().getX() < 5.5){
        target = new Translation2d(4.62, 4.03);
      } else if(drivetrain.getSwerveDrive().getPose().getY() >= 4.03){
        target = new Translation2d(1, 5);
      } else if (drivetrain.getSwerveDrive().getPose().getY() < 4.03) {
        target = new Translation2d(1, 3);
      }
    }
    if (DriverStation.getAlliance().get() == Alliance.Red){
      // hub = new Translation2d(15.13, 13.19);
      if(drivetrain.getSwerveDrive().getPose().getX() > 13){
        target = new Translation2d(12, 4.03);
      } else if(drivetrain.getSwerveDrive().getPose().getY() >= 4.03){
        target = new Translation2d(14.5, 5);
      } else if (drivetrain.getSwerveDrive().getPose().getY() < 4.03) {
        target = new Translation2d(14.5, 3);
      }
      
    }

    double targetTOF = 1.15;
    Translation2d targetPose = target.minus(new Translation2d(drivetrain.getSwerveDrive().getFieldVelocity().vxMetersPerSecond,drivetrain.getSwerveDrive().getFieldVelocity().vyMetersPerSecond).times(targetTOF));
    
    // Translation2d targetPose = hub;


    //  Transform2d transformToHub = drivetrain.getSwerveDrive().getPose()
    // Transform2d transformToHub = turret.getTurretPose();
    Translation2d toTarget = targetPose.minus(turret.getTurretPose().getTranslation());
    // double distanceToHub = Math.sqrt(Math.pow(transformToHub.getX(), 2)+Math.pow(transformToHub.getY(), 2));
    double distanceToTarget = toTarget.getNorm() ;

    Rotation2d fieldAngleToTarget = toTarget.getAngle();

    // desiredTurretAngle = fieldAngleToTarget.minus(turret.getTurretRotation());

    // turret.setTurretAngle(desiredTurretAngle);

    // distanceToTarget -= 1;

    hood.setHoodAngleFunc(ShooterLookupTables.hoodAngleMap.get(distanceToTarget));
    // outtake.setTargetVelocity(flywheelSpeedMap.get(distanceToTarget));
    flywheel.setTargetVelocity(ShooterLookupTables.flywheelSpeedMap.get(distanceToTarget));
    turret.setTurretGlobalAngle(fieldAngleToTarget.getDegrees());
    SmartDashboard.putNumber("Hub Distance", distanceToTarget);
    SmartDashboard.putNumber("Hood Table", ShooterLookupTables.hoodAngleMap.get(distanceToTarget));
    SmartDashboard.putNumber("Flywheel Table", ShooterLookupTables.flywheelSpeedMap.get(distanceToTarget));

    if (flywheel.atSetpoint() && turret.isTurretAligned()){
      kicker.Spindexer.set(-0.5);
      kicker.kicker.set(0.4);
    }
  }

  public Rotation2d getDesiredAngle(){
    return desiredTurretAngle;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    flywheel.setTargetVelocity(0);
    kicker.Spindexer.set(0);
    kicker.kicker.set(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}