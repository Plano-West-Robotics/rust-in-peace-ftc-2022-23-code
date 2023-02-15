package org.firstinspires.ftc.teamcode.configs;

import static org.firstinspires.ftc.teamcode.configs.StartingTiles.StartTile.*;

import com.acmerobotics.roadrunner.geometry.Pose2d;

public class StartingTiles {
    public enum StartTile {A2, A5, F2, F5}


    /**
     * shifts the pose to the correct one for field centric drive given the final pose and the starting tile
     * @param pose the final pose
     * @param startTile the starting tile
     * @return the shifted pose
     */
    public static Pose2d shiftPoseAngle(Pose2d pose, StartTile startTile){
        int shifter = 0;
        if (startTile == A2 || startTile == A5){
            shifter = -90;
        }
        if (startTile == F2 || startTile == F5){
            shifter = 90;
        }

        double newAngle = Math.toDegrees(pose.getHeading())-shifter;

        return new Pose2d(pose.getX(), pose.getY(), Math.toRadians(newAngle));
    }

    public static Pose2d storage = new Pose2d();

}
