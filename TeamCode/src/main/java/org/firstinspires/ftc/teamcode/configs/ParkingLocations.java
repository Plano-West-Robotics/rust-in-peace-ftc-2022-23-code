package org.firstinspires.ftc.teamcode.configs;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;

@Config
public class ParkingLocations {
    public static Pose2d F2Point1 = new Pose2d(-45, -9, Math.toRadians(-90));
    //public static Pose2d F2Point2 = new Pose2d(-45, -27);

    public static Pose2d F5Point1 = new Pose2d(9, -9, Math.toRadians(-90));
    //public static Pose2d F5Point2 = new Pose2d(9, -27);

    public static Pose2d A2Point1 = new Pose2d(-45, 9, Math.toRadians(90));
    //public static Pose2d A2Point2 = new Pose2d(-45, 27);

    public static Pose2d A5Point1 = new Pose2d(9, 9, Math.toRadians(90));
    //public static Pose2d A5Point2 = new Pose2d(9, 27);

    public static double FLineY = -9;
    public static double ALine = 9;

}
