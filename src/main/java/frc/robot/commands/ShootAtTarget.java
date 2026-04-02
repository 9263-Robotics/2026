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
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Kicker;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

/** An example command that uses an example subsystem. */
public class ShootAtTarget extends Command {
  @SuppressWarnings("PMD.UnusedPrivateField")

  private static final InterpolatingDoubleTreeMap hoodAngleMap = new InterpolatingDoubleTreeMap();

  private static final InterpolatingDoubleTreeMap flywheelSpeedMap = new InterpolatingDoubleTreeMap();
  
    static { // x is distance, y is angle
      hoodAngleMap.put(1.0,1.0);
      hoodAngleMap.put(2.0,4.0);
      hoodAngleMap.put(3.0,9.0);
      

      flywheelSpeedMap.put(1.0, 5000.0);
      flywheelSpeedMap.put(2.0, 5000.0);
      flywheelSpeedMap.put(3.0, 5000.0);
      flywheelSpeedMap.put(4.5, 6700.0);

  }

  private final Turret turret;
  private final Kicker outtake;
  private final Hood hood;


  Rotation2d desiredTurretAngle = null;


  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
   public ShootAtTarget(Turret turret, Kicker outtake, Hood hood) {
    this.turret = turret;
    this.outtake = outtake;
    this.hood = hood;

    addRequirements(turret);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Translation2d hub = null;
    // m_turret.
    if (DriverStation.getAlliance().get() == Alliance.Red){
      hub = new Translation2d(39.05, 13.19);
    }
    if (DriverStation.getAlliance().get() == Alliance.Blue){
      hub = new Translation2d(15.13, 13.19);
    }

    //  Transform2d transformToHub = drivebase.getSwerveDrive().getPose().minus(hub);
    // Transform2d transformToHub = turret.getTurretPose();
    Translation2d toHub = hub.minus(turret.getTurretPose().getTranslation());
    // double distanceToHub = Math.sqrt(Math.pow(transformToHub.getX(), 2)+Math.pow(transformToHub.getY(), 2));
    double distanceToHub = toHub.getNorm();

    Rotation2d fieldAngleToHub = toHub.getAngle();

    desiredTurretAngle = fieldAngleToHub.minus(turret.getTurretRotation());

    turret.setTurretAngle(desiredTurretAngle);

    // hood.setHoodAngle(hoodAngleMap.get(distanceToHub));

    // outtake.setTargetVelocity(flywheelSpeedMap.get(distanceToHub));

    // if (outtake.atSetpoint() && turret.isTurretAligned()){
    //   outtake.Spindexer.set(-0.5);
    //   outtake.kicker.set(0.4);
    // }
  }

  public Rotation2d getDesiredAngle(){
    return desiredTurretAngle;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    outtake.kicker.set(0);
    outtake.Spindexer.set(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}