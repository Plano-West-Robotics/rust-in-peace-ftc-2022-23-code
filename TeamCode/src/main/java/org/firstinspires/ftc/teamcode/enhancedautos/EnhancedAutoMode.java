package org.firstinspires.ftc.teamcode.enhancedautos;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.driveobjs.ActionObject;
import org.firstinspires.ftc.teamcode.driveobjs.EnhancedDriver;
import org.firstinspires.ftc.teamcode.driveobjs.aprilTag.AprilTagDetector;

import static org.firstinspires.ftc.teamcode.configs.ParkingLocations.*;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

public abstract class EnhancedAutoMode extends LinearOpMode {
    private EnhancedDriver enhancedDriver;
    private FtcDashboard dashboard = FtcDashboard.getInstance();
    private Telemetry dashboardTelemetry = dashboard.getTelemetry();
    enum StartTile {A5, A2, F2, F5}

    public List<ActionObject> actionObjects =  new ArrayList<>(0);
    private StartTile startTile;


    public void run(){


        enhancedDriver.run(actionObjects);

        Pose2d currentPosition = enhancedDriver.getPoseEstimate();
        double currentX = currentPosition.getX();
        double currentY = currentPosition.getY();

    }


    public void initThings(Pose2d startPos, StartTile startTile, ActionObject[] actionObjects, int parkPosition){
        enhancedDriver = new EnhancedDriver(hardwareMap);
        enhancedDriver.initPosition(startPos);
        this.actionObjects = new ArrayList<ActionObject>(Arrays.asList(actionObjects));
        calculateParking(startTile, this.actionObjects.get(this.actionObjects.size()-1), parkPosition);
    }

    public List<ActionObject> calculateParking(StartTile startTile, ActionObject lastPosition, int parkPosition){
        //TODO
        Pose2d lastPose = lastPosition.getPose2d();
        double lastX = lastPose.getX();
        double lastY = lastPose.getY();
        double angle = lastPose.getHeading();



        //creates the line that represents valid parking spots
        Pose2d goalLinePoint1 = null;
        Pose2d goalLinePoint2 = null;
        switch(startTile) {
            case A2:
                goalLinePoint1 = A2Point1;
                goalLinePoint2 = A2Point2;
                break;
            case A5:
                goalLinePoint1 = A5Point1;
                goalLinePoint2 = A5Point2;
                break;
            case F2:
                goalLinePoint1 = F2Point1;
                goalLinePoint2 = F2Point2;
                break;
            case F5:
                goalLinePoint1 = F5Point1;
                goalLinePoint2 = F5Point2;
                break;
        }

        //offsets those lines based on the parking position
        switch(parkPosition){
            case(3):
                goalLinePoint1 = new Pose2d(goalLinePoint1.getX()+18, goalLinePoint1.getY()+18);
                goalLinePoint2 = new Pose2d(goalLinePoint2.getX()+18, goalLinePoint2.getY()+18);
            case(2):
                goalLinePoint1 = new Pose2d(goalLinePoint1.getX()+18, goalLinePoint1.getY()+18);
                goalLinePoint2 = new Pose2d(goalLinePoint2.getX()+18, goalLinePoint2.getY()+18);
            case(1):
                break;
        }

        for (double i = 0; i < 26; i+=0.1){
            double newX = angle*Math.cos(angle+180);
            double newY = angle*Math.sin(angle+180);

            double[] x = new double[]{newX, goalLinePoint1.getX(), goalLinePoint2.getX()};
            double[] y = new double[]{newY, goalLinePoint1.getY(), goalLinePoint2.getY()};

            

        }



        if(Math.abs(lastX % 6) < Math.abs(lastY % 6)){

        }
        if(Math.abs(lastY % 6) < Math.abs(lastX % 6)){

        }








        return null;
    }


    public int readAprilTag(){
        AprilTagDetector detector = new AprilTagDetector(hardwareMap);
        int pos = detector.getPos();
        detector.endStream();

        //store it in the form of a Pose2D to parkLocation if you would
        pos = 0;

        return pos;
    }

}
