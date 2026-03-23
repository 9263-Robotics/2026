// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.interpolation.*;

import java.util.ArrayList;

import org.dyn4j.geometry.Vector2;

public class DistanceToAngleMap {

    // Map: distance (meters) → Angle (degrees)
    private final InterpolatingTreeMap<Double, Double> map = new InterpolatingDoubleTreeMap();
    
    public DistanceToAngleMap(String filePath) {
        // Read the file and populate the map
        ArrayList<Vector2> values = VectorFileReader.getVectors(filePath);
        System.out.println(values);

        populateMap(values);
    }

    public double getAngleForDistance(double distanceMeters) {

        Double result = map.get(distanceMeters);

        if (result == null) {
            System.err.println("DistanceToAngleMap returned null for distance: " + distanceMeters);
            return 0.0;
        }

        return result;
    }

    private void populateMap(ArrayList<Vector2> values){
        for (Vector2 v : values) {
            map.put(v.x, v.y);
        }
    } 
}