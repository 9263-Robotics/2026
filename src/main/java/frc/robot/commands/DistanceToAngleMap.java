// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.InterpolatingDouble;

import java.util.ArrayList;

public class DistanceToAngleMap {

    // Map: distance (meters) → Angle (degrees)
    private final InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble> map =
        new InterpolatingTreeMap<>();
    
    public DistanceToAngleMap(String filePath) {
        // Read the file and populate the map
        ArrayList<Vector2> values = VectorFileReader.getVectors(filePath);
        System.out.println(values);

        populateMap(values);
    }

    public double getAngleForDistance(double distanceMeters) {
        if (map.isEmpty()) {
            System.err.println("DistanceToAngleMap is empty!");
            return 0.0;
        }

        InterpolatingDouble result = map.getInterpolated(new InterpolatingDouble(distanceMeters));

        if (result == null) {
            System.err.println("DistanceToAngleMap returned null for distance: " + distanceMeters);
            return 0.0;
        }

        return result.value;
    }

    private void populateMap(ArrayList<Vector2> values){
        for (Vector2 v : values) {
            map.put(new InterpolatingDouble(v.x), new InterpolatingDouble(v.y));
        }
    } 
}