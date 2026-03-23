// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
//import frc.robot.subsystems.TurretSubsystem;

import frc.robot.commands.DistanceToAngleMap;
import frc.robot.commands.AnglePair;
import frc.robot.commands.VectorFileReader;

import java.io.BufferedReader;
import java.io.FileReader;
import java.io.IOException;

public class TurretLineupCommand extends SubsystemBase {
  /** Creates a new TurretLineup. */
  private final DistanceToAngleMap distanceToAngleMap = new DistanceToAngleMap("/home/lvuser/vector_list.txt");
  //private final TurretSubsystem turretSubsystem = new TurretSubsystem(1);

  public TurretLineupCommand() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  public AnglePair getAngles(Pose2d target, Pose2d robot, ChassisSpeeds robotVelocity)
  {
        double robot_displacement_x = robotVelocity.vxMetersPerSecond * 0.02; // Assuming a 20ms loop time
        double robot_displacement_y = robotVelocity.vyMetersPerSecond * 0.02; // Assuming a 20ms loop time 
        
        double turret_vertical_angle = HoodAngleCalculate(target.getTranslation().getDistance(robot.getTranslation()));

        double dx = robot.getTranslation().getX() - robot_displacement_x;
        double dy = robot.getTranslation().getY() - robot_displacement_y;

        double turret_horizontal_angle = HorizontalAngleCalculate(dx, dy);
        
        return new AnglePair(turret_vertical_angle, turret_horizontal_angle);
  }

  public double HoodAngleCalculate(double distance) {
    return distanceToAngleMap.getAngleForDistance(distance);
  }

  public double HorizontalAngleCalculate(double x, double y) {
    return 90 - Math.toDegrees(Math.atan2(x, y));
  }

  public void setAngles(Pose2d target, Pose2d robot, ChassisSpeeds robotVelocity) {
    // Example usage with dummy values
    AnglePair angles = getAngles(target, robot, robotVelocity);
    //turretSubsystem.setTurretVerticalAngle(angles.verticalAngle);
    //turretSubsystem.setTurretHorizontalAngle(angles.horizontalAngle);
  }
}