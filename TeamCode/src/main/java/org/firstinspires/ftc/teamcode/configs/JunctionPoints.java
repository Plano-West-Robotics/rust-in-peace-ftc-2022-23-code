package org.firstinspires.ftc.teamcode.configs;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;

import java.util.ArrayList;

@Config
public class JunctionPoints {
    public static ArrayList<Pose2d> generateJunctionPoints(){
        ArrayList<Pose2d> junctions = new ArrayList<>();
        for (int i = 0; i < 5; i++){
            for(int j = 0; j < 5; j++){
                junctions.add(new Pose2d(-36+(18*i), -36+(18*j) ));
            }
        }
        return junctions;
    }



}
