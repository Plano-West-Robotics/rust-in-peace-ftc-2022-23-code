package org.firstinspires.ftc.teamcode.enhancedautos;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.driveobjs.ActionObject;
import org.firstinspires.ftc.teamcode.driveobjs.EnhancedDriver;
import org.firstinspires.ftc.teamcode.driveobjs.ObjectDetector;
import org.firstinspires.ftc.teamcode.driveobjs.aprilTag.AprilTagDetector;

import java.util.ArrayList;
import java.util.List;

public abstract class EnhancedAutoMode extends LinearOpMode {
    private EnhancedDriver enhancedDriver;
    private FtcDashboard dashboard = FtcDashboard.getInstance();
    private Telemetry dashboardTelemetry = dashboard.getTelemetry();
    public List<ActionObject> actionObjects =  new ArrayList<>(0);
    public Pose2d parkLocation;

    public void run(){
        enhancedDriver.run(actionObjects);

        Pose2d currentPosition = enhancedDriver.getPoseEstimate();

    }


    public void initThings(){
        enhancedDriver = new EnhancedDriver(hardwareMap);
    }

    public int readAprilTag(){
        //TODO: @Varun needs to implement this
        AprilTagDetector detector = new AprilTagDetector(hardwareMap);
        int pos = detector.getPos();
        detector.endStream();

        //store it in the form of a Pose2D to parkLocation if you would
        parkLocation = null;

        return pos;
    }

}
