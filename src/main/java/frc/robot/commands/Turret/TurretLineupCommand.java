// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Turret;
import java.util.Arrays;
import java.util.List;
import java.util.ArrayList;
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
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Kicker;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import swervelib.parser.json.modules.AngleConversionFactorsJson;
import org.dyn4j.geometry.Vector2;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.io.BufferedReader;
import java.io.FileReader;
import java.io.IOException;

public class TurretLineupCommand extends Command {

  private static final InterpolatingDoubleTreeMap hoodAngleMap =  new InterpolatingDoubleTreeMap();
  //("/home/lvuser/vector_list.txt");

  private static final InterpolatingDoubleTreeMap flywheelSpeedMap = new InterpolatingDoubleTreeMap();
  private static ArrayList<Vector2> hoodvalues = new ArrayList<Vector2>();
  private static ArrayList<Vector2> flywheelvalues = new ArrayList<Vector2>();
  
  static { 
    hoodvalues = FileRead.getFile("/home/lvuser/hood_vector_list.txt");
    flywheelvalues = FileRead.getFile("/home/lvuser/flywheel_vector_list.txt");
    for (Vector2 h : hoodvalues) {
            hoodAngleMap.put(h.x, h.y);
    }
    for (Vector2 f : flywheelvalues) {
            flywheelSpeedMap.put(f.x, f.y);
    }
  }

  private final SwerveSubsystem drivebase;
  private final Turret turret;
  private final Hood hood;
  private final Kicker kicker;
  
  Rotation2d desiredTurretAngle = null;

  /** Creates a new TurretLineup. */
  public TurretLineupCommand(SwerveSubsystem drivebase, Turret turret, Hood hood, Kicker kicker) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.turret = turret;
    this.drivebase = drivebase;
    this.hood = hood;
    this.kicker = kicker;
  }

  public double getHoodAngle(double distanceMeters) {
        Double predictedHoodAngle = hoodAngleMap.get(distanceMeters);
        if (predictedHoodAngle == null) {
            System.err.println("hoodAngleMap returned null for distance: " + distanceMeters);
            return 0.0;
        }
        return predictedHoodAngle;
  }

  public double getFlywheelSpeed(double distanceMeters) {
        Double predictedFlywheelSpeed = flywheelSpeedMap.get(distanceMeters);
        if (predictedFlywheelSpeed == null) {
            System.err.println("flywheelSpeedMap returned null for distance: " + distanceMeters);
            return 0.0;
        }
        return predictedFlywheelSpeed;
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

    double dx = turret.getTurretPose().getX() + drivebase.getSwerveDrive().getRobotVelocity().vxMetersPerSecond*0.05;
    double dy = turret.getTurretPose().getY() + drivebase.getSwerveDrive().getRobotVelocity().vyMetersPerSecond*0.05;

    // Transform2d transformToHub = drivebase.getSwerveDrive().getPose().minus(hub);
    // Transform2d transformToHub = turret.getTurretPose();
    Translation2d toHub = hub.minus(new Translation2d(
          turret.getTurretPose().getX() + drivebase.getSwerveDrive().getRobotVelocity().vxMetersPerSecond*0.05, 
          turret.getTurretPose().getY() + drivebase.getSwerveDrive().getRobotVelocity().vyMetersPerSecond*0.05));
    // double distanceToHub = Math.sqrt(Math.pow(transformToHub.getX(), 2)+Math.pow(transformToHub.getY(), 2));
    double distanceToHub = toHub.getNorm();

    double predictedVerticalAngle = hoodAngleMap.get(distanceToHub);
    double turret_horizontal_angle = 90 - Math.toDegrees(Math.atan2(dx, dy));

    double predictedFlywheelSpeed = flywheelSpeedMap.get(distanceToHub);
    Rotation2d fieldAngleToHub = toHub.getAngle();

    desiredTurretAngle = fieldAngleToHub.minus(turret.getTurretRotation());

    turret.setTurretAngle(desiredTurretAngle);
    hood.setHoodAngle(predictedVerticalAngle);
    kicker.shoot(predictedFlywheelSpeed);
  }

  public Rotation2d getDesiredAngle(){
    return desiredTurretAngle;
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