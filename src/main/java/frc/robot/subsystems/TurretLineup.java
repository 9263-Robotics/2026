// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.InterpolatingDouble;

import java.io.BufferedReader;
import java.io.FileReader;
import java.io.IOException;


public class TurretLineup extends SubsystemBase {
  /** Creates a new TurretLineup. */
  private final DistanceToAngleMap distanceToAngleMap = new DistanceToAngleMap("/home/lvuser/vector_list.txt");

  public TurretLineup() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  public AnglePair getAngles(
    double target_x, double target_y, double target_z, 
    double robot_x, double robot_y, double robot_z, 
    double robot_velocity_x, double robot_velocity_y)
  {
        double camera_distance = Math.sqrt(
          (target_x-robot_x)*(target_x-robot_x) + 
          (target_y-robot_y)*(target_y-robot_y) + 
          (target_z-robot_z)*(target_z-robot_z));
        double robot_displacement_x = robot_velocity_x * 0.02; // Assuming a 20ms loop time
        double robot_displacement_y = robot_velocity_y * 0.02; // Assuming a 20ms loop time 
        
        double turret_vertical_angle = VerticalAngleCalculate(camera_distance);

        double dx = robot_x - robot_displacement_x;
        double dy = robot_y - robot_displacement_y;

        double turret_horizontal_angle = HorizontalAngleCalculate(dx, dy);
        
        return new AnglePair(turret_vertical_angle, turret_horizontal_angle);
  }

  public double VerticalAngleCalculate(double distance) {
    return distanceToAngleMap.getAngleForDistance(distance);
  }

  public double HorizontalAngleCalculate(double x, double y) {
    return 90 - Math.toDegrees(Math.atan2(x, y));
  }
}