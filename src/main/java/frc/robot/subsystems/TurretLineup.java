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

public class ReadTextFile {
    public static void main(String[] args) {
        // Change this to your file path
        String filePath = "vector_list.txt";
        // Read the file
        getVectors(filePath);
    }

    /**
     * Reads and prints the contents of a text file line by line.
     * @param filePath Path to the text file
     */
    public static void getVectors(String filePath) {
        // Try-with-resources ensures the file is closed automatically
        try (BufferedReader br = new BufferedReader(new FileReader(filePath))) {
            String line;
            // List to store the vectors read from the file
            ArrayList<Vector2> vectors = new ArrayList<>();
            boolean isEmpty = true;
            while ((line = br.readLine()) != null) {
                System.out.println(line);
                isEmpty = false;
                String[] parts = line.split(",");
                if (parts.length == 2) {
                    try {
                        double distance = Double.parseDouble(parts[0].trim());
                        double angle = Double.parseDouble(parts[1].trim());
                        vectors.add(new Vector2(distance, angle));
                    } catch (NumberFormatException e) {
                        System.err.println("[ERROR] Invalid number format in line: " + line);
                    }
                } else {
                    System.err.println("[ERROR] Invalid line format (expected 'x,y'): " + line);
                }
            }
            if (isEmpty) {
                System.out.println("[INFO] The file is empty.");
            }
        } catch (IOException e) {
            System.err.println("[ERROR] Unable to read file: " + e.getMessage());
        }
    }
}
public class InterpolatingTreeMap_DistanceToAngle {

    ArrayList<Vector2> values = new ArrayList<>();
    values = getVectors("vector_list.txt");
    System.out.println(values);

    // Map: distance (meters) → Angle (degrees)
    private final InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble> distanceToAngle =
        new InterpolatingTreeMap<>();
    
    public InterpolatingTreeMap_DistanceToAngle() {
        treehelper(distanceToAngle, values);
    }

    public double getAngleForDistance(double distanceMeters) {

        return distanceToAngle.getInterpolated(new InterpolatingDouble(distanceMeters)).value;
    }
}


void treehelper(InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble> map, ArrayList<Vector2> values){
    for (Vector2 value : values) {
        double distance = value.x; // Assuming x is distance
        double angle = value.y;    // Assuming y is angle
        map.put(new InterpolatingDouble(distance), new InterpolatingDouble(angle));
    }
}


public class TurretLineup extends SubsystemBase {
  /** Creates a new TurretLineup. */
  public Command TurretLineup(double target_x, double target_y, double target_z, double robot_x, double robot_y, double robot_z, double robot_velocity_x, double robot_velocity_y) {
    double camera_distance = Math.sqrt((target_x-robot_x)*(target_x-robot_x) + (target_y-robot_y)*(target_y-robot_y) + (target_z-robot_z)*(target_z-robot_z));
    double robot_displacement_x = robot_velocity_x * 0.02; // Assuming a 20ms loop time
    double robot_displacement_y = robot_velocity_y * 0.02; // Assuming a 20ms loop time
    return runOnce(
        () -> { 
          double turret_vertical_angle = VerticalAngle(camera_distance);
          double turret_horizontal_angle = HorizontalAngle(robot_x - robot_displacement_x, robot_y - robot_displacement_y);
        });
  }
  public double VerticalAngle(double distance) {
    InterpolatingTreeMap_DistanceToAngle distanceToAngleMap = new InterpolatingTreeMap_DistanceToAngle();
    double turret_vertical_angle = distanceToAngleMap.getAngleForDistance(distance);
    return turret_vertical_angle;
  }

  public double HorizontalAngle(double x, double y) {
    double camera_distance = Math.sqrt(x*x + y*y);
    double theta = Math.atan(x/y);
    double turret_horizontal_angle = 90 - theta; 
    return turret_horizontal_angle;
  }
}