package frc.robot.commands;

import edu.wpi.first.wpilibj.Filesystem;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.io.IOException;
import java.util.ArrayList;

import org.dyn4j.geometry.Vector2;

public class VectorFileReader {

    public static ArrayList<Vector2> getVectors(String filename) {

        ArrayList<Vector2> vectors = new ArrayList<>();

        File deployFile = new File(
            Filesystem.getDeployDirectory(), filename
        );

        try (BufferedReader br = new BufferedReader(new FileReader(deployFile))) {
            String line;
            while ((line = br.readLine()) != null) {
                String[] parts = line.split(",");
                if (parts.length == 2) {
                    double distance = Double.parseDouble(parts[0].trim());
                    double angle = Double.parseDouble(parts[1].trim());
                    vectors.add(new Vector2(distance, angle));
                }
            }
        } catch (IOException e) {
            System.err.println("File read error: " + e.getMessage());
        }

        return vectors;
    }
}