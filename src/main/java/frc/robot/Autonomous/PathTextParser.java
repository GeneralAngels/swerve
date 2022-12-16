// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Autonomous;

import java.io.IOException;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Paths;
import java.util.Collections;
import java.util.List;

/** Add your docs here. */
public class PathTextParser {
    String text;
    List<String> lines;
    
    public PathTextParser(String path) {
        lines = this.readFileInList(path);
    }

    public double[][] getPathArray() {
        double[][] path = new double[lines.size()][7];
        for (int i = 0; i < path.length; i++) {
            String[] splitted = lines.get(i).split(",");
            double[] values = new double[7];
            
            for (int j = 0; j < values.length; j++) {
                values[j] = Double.parseDouble(splitted[j]);
            }

            path[i] = values;
        }

        return path;
    }

    public static List<String> readFileInList(String path)
    {
    
        List<String> lines = Collections.emptyList();
        try
        {
        lines =
        Files.readAllLines(Paths.get(path), StandardCharsets.UTF_8);
        }
    
        catch (IOException e)
        {
    
        // do something
        e.printStackTrace();
        }
        return lines;
    }
}
