// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Turret;

import java.util.Arrays;
import java.util.List;
import java.util.Map;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.InverseInterpolator;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

/** An example command that uses an example subsystem. */
public class ShootAtTarget extends Command {
  @SuppressWarnings("PMD.UnusedPrivateField")

  private static final InterpolatingDoubleTreeMap table = new InterpolatingDoubleTreeMap();
  
    static {
      table.put(1.0,1.0);
      table.put(2.0,4.0);
      table.put(3.0,9.0);
  }

  private final SwerveSubsystem drivebase;
  private final Turret turret;


  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
   public ShootAtTarget(Turret turret, SwerveSubsystem drivebase) {
    this.turret = turret;
    this.drivebase = drivebase;

    addRequirements(turret);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Pose2d hub = null;
    // m_turret.
   if (DriverStation.getAlliance().get() == Alliance.Red){
      hub = new Pose2d(39.05, 13.19, new Rotation2d());
   }
    if (DriverStation.getAlliance().get() == Alliance.Blue){
      hub = new Pose2d(15.13, 13.19, new Rotation2d());
   }
   Transform2d transformToHub = drivebase.getSwerveDrive().getPose().minus(hub);
   double distanceToHub = Math.sqrt(Math.pow(transformToHub.getX(), 2)+Math.pow(transformToHub.getY(), 2));
   double predictedAngle = table.get(distanceToHub);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}