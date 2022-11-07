package org.firstinspires.ftc.teamcode.enhancedautos;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.driveobjs.ActionObject;
import org.firstinspires.ftc.teamcode.driveobjs.EnhancedDriver;
import org.firstinspires.ftc.teamcode.driveobjs.aprilTag.AprilTagDetector;

import static org.firstinspires.ftc.teamcode.configs.JunctionPoints.generateJunctionPoints;
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
        List<ActionObject> newLocations = calculateParking(startTile, this.actionObjects, parkPosition);
        for (ActionObject newLocation : newLocations){
            this.actionObjects.add(newLocation);
        }
    }

    public ArrayList<ActionObject> calculateParking(StartTile startTile, List<ActionObject> actionObjects, int parkPosition){
        //TODO: Everything

        ArrayList<ActionObject> newLocations = new ArrayList<>(0);
        ActionObject lastPosition = actionObjects.get(actionObjects.size()-1);


        Pose2d lastPose = lastPosition.getPose2d();
        double lastX = lastPose.getX();
        double lastY = lastPose.getY();
        double lastHeading = lastPose.getHeading();

        //gets the actual Pose2d representing the ideal parking location
        Pose2d parkingGoal = calculateTargetPositions(startTile, parkPosition);

        Pose2d[] collisionSquare = calculateCollisionSquarePoints(lastHeading, lastPose);

        List<Pose2d> collisionPoints = checkForCollison(collisionSquare, lastPose);

        if (!collisionPoints.isEmpty()){
            dashboardTelemetry.addLine("Collision Detected");
            dashboardTelemetry.update();
            //TODO: Get out of the collision and realign
        }

        /**
         * calculates the nearest center of tile
         */
        //the x value change in the negative direction
        double closeNegX = -1*((lastPose.getX()+9) % 18);
        //the x value change in the positive direction
        double closePosX = 18 - ((lastPose.getX()+9) % 18);

        //the y value change in the negative direction
        double closeNegY = -1*((lastPose.getY()+9) % 18);
        //the y value change in the positive direction
        double closePosY = 18 - ((lastPose.getY()+9) % 18);

        Pose2d nearestCenter = new Pose2d(
                //checks for the smaller of the two distances and adds that to the last position
                lastX + (Math.abs(closeNegX) < closePosX ? closeNegX : closePosX),
                lastX + (Math.abs(closeNegY) < closePosY ? closeNegY : closePosY),
                lastHeading
                );
        //adds the nearest center to the list, once to get there, another time to readjust heading
        newLocations.add(new ActionObject(nearestCenter));

        /**
         * first checks to see if it is already in position
         * if so, just adjusts the heading accordingly and returns the locations
         */

        if (Math.abs(nearestCenter.getX() - parkingGoal.getX()) < 4 && Math.abs(nearestCenter.getY() - parkingGoal.getY()) < 4) {
            newLocations.add(new ActionObject(parkingGoal));
            return newLocations;
        }


        /**
         * moves onto the "rail line" first after turning to the correct orientation
         */
        Pose2d lastNewPose = newLocations.get(newLocations.size()-1).getPose2d();
        newLocations.add(new ActionObject(new Pose2d(lastNewPose.getX(), lastNewPose.getY(), parkingGoal.getHeading())));
        newLocations.add(new ActionObject(new Pose2d(lastNewPose.getX(), parkingGoal.getY(), parkingGoal.getHeading())));
        newLocations.add(new ActionObject(parkingGoal));


        return newLocations;
    }

    private ArrayList<Pose2d> checkForCollison(Pose2d[] square, Pose2d lastPose) {
        ArrayList<Pose2d> junctions = generateJunctionPoints();
        ArrayList<Pose2d> collidingJunctions = new ArrayList<Pose2d>(0);
        for(Pose2d junction : junctions){
            /**
             * This code generates a list of vectors between each junction and the corners
             */
            List<Vector2d> vector2dList = new ArrayList<Vector2d>(0);
            for(Pose2d corner : square) {
                vector2dList.add(new Vector2d(corner.getX()-junction.getX(), corner.getY()- junction.getY()));
            }
            /**
             * This code summs the angle between the vectors
             */
            double totalAngle = 0;
            for(int i = 0; i < 4; i++){
                totalAngle+=vector2dList.get(i).angleBetween(vector2dList.get((i+1)%4));
            }
            /**
             * this should be a good approximation of collision
             */
            if (totalAngle > 355){
                collidingJunctions.add(junction);
            }
        }
        return collidingJunctions;
    }

    private Pose2d[] calculateCollisionSquarePoints(double angle, Pose2d lastPose) {
        angle+=45;
        Pose2d[] square = new Pose2d[4];
        for (int i = 0; i < 4; i++){
            angle%=360;
            square[i] = new Pose2d(
                    9*Math.sqrt(2)*Math.cos(angle)+lastPose.getX(),
                    9*Math.sqrt(2)*Math.sin(angle)+lastPose.getY());
            angle+=90;
        }
        return square;
    }

    private Pose2d calculateTargetPositions(StartTile startTile, int parkPosition) {
        Pose2d parkingGoal = null;
        switch(startTile) {
            case A2:
                parkingGoal = A2Point1;
                break;
            case A5:
                parkingGoal = A5Point1;
                break;
            case F2:
                parkingGoal = F2Point1;
                break;
            case F5:
                parkingGoal = F5Point1;
                break;
        }

        //offsets those lines based on the parking position
        switch(parkPosition){
            case(3):
                parkingGoal = new Pose2d(parkingGoal.getX()+18, parkingGoal.getY()+18);
            case(2):
                parkingGoal = new Pose2d(parkingGoal.getX()+18, parkingGoal.getY()+18);
            case(1):
                break;
        }

        return parkingGoal;
    }


    public int readAprilTag(){
        AprilTagDetector detector = new AprilTagDetector(hardwareMap);
        int pos = detector.getPos();
        detector.endStream();

        //store it in the form of a Pose2D to parkLocation if you would
        return pos;
    }

}
