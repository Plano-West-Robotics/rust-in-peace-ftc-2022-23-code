package org.firstinspires.ftc.teamcode.enhancedautos;


import static org.firstinspires.ftc.teamcode.configs.JunctionPoints.generateJunctionPoints;
import static org.firstinspires.ftc.teamcode.configs.ParkingLocations.A2Point1;
import static org.firstinspires.ftc.teamcode.configs.ParkingLocations.A5Point1;
import static org.firstinspires.ftc.teamcode.configs.ParkingLocations.F2Point1;
import static org.firstinspires.ftc.teamcode.configs.ParkingLocations.F5Point1;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.configs.PoseStorage;
import org.firstinspires.ftc.teamcode.driveobjs.ActionObject;
import org.firstinspires.ftc.teamcode.driveobjs.ActionObjectOld;
import org.firstinspires.ftc.teamcode.driveobjs.DriverMethodQueue;
import org.firstinspires.ftc.teamcode.driveobjs.drivers.EnhancedDriver;
import org.firstinspires.ftc.teamcode.driveobjs.aprilTag.AprilTagDetector;

import java.lang.reflect.Method;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

public abstract class EnhancedAutoMode extends LinearOpMode {
    protected EnhancedDriver enhancedDriver;
    private FtcDashboard dashboard = FtcDashboard.getInstance();
    private Telemetry dashboardTelemetry = dashboard.getTelemetry();
    enum StartTile {A5, A2, F2, F5}
    AprilTagDetector detector;
    Method[] enhancedDriverMethods;
    DriverMethodQueue queue;

    public List<ActionObjectOld> actionObjects =  new ArrayList<>(0);
    //private StartTile startTile;


    public void run(){

        enhancedDriver.moveConeOutOfWay();

        enhancedDriver.run(actionObjects);


        PoseStorage.currentPose = actionObjects.get(actionObjects.size()-1).getPose2d();
    }

    public void runQueue() {
        for (int i = 0; i < queue.length(); i++) {
            ActionObject actionObject = queue.getActionObject(i);
            boolean beenExecuted = false;

            //cycles through all the methods and if there is a match for the method, invokes it
            for (Method method : enhancedDriverMethods) {

                if (checkMatch(method, actionObject)) {
                    //dumps args into a new array that puts the enhancedDriver at the front
                    Object[] originalArgs = actionObject.getArgs();
                    Object[] args = new Object[originalArgs.length + 1];
                    args[0] = enhancedDriver;
                    for (int j = 1; j < args.length; j++) {
                        args[j] = originalArgs[j - 1];
                    }
                    try {
                        beenExecuted = true;
                        method.invoke(args);
                    } catch (Exception e) {
                        dashboardTelemetry.addLine("ERROR WITH METHOD CALL");
                        dashboardTelemetry.addLine(e.getMessage());
                        dashboardTelemetry.update();
                        sleep(10000);


                    }
                }
                if (!beenExecuted) {
                    dashboardTelemetry.addLine("Method was not executed");
                    dashboardTelemetry.addLine("Method: " + method.getName());
                    dashboardTelemetry.update();
                    sleep(10000);
                }
            }
        }
    }

    /**
     * this code should check if the method matched the method requested in actionObject
     * @param method the method that we are checking
     * @param actionObject the actionObject that contains the method and args
     * @return returns true if they match, else returns false
     */
    public boolean checkMatch(Method method, ActionObject actionObject){
        if (method.getName().equals(actionObject.getMethodName()))
            return false;

        Class<?>[] parameters =  method.getParameterTypes();
        Object[] args = actionObject.getArgs();

        //first does the obvious length match
        if (parameters.length != args.length)
            return false;

        //please work please work please work
        for (int i = 0; i < parameters.length; i++){
            Object argument = args[i];
            Class<?> parameterType = parameters[i];
            if (!parameterType.isInstance(argument)){
                return false;
            }
        }
        return true;
    }




    // basic initialization, does not calulate the rest of the parking path
    public void initThings(Pose2d startPos, StartTile startTile, ActionObjectOld[] actionObjects){
        enhancedDriver = new EnhancedDriver(hardwareMap);
        enhancedDriver.initPosition(startPos);
        this.actionObjects = new ArrayList<ActionObjectOld>(Arrays.asList(actionObjects));
    }

    // implements the calculate parking to automatic calculate the parking position from the final position
    // TODO: DOES NOT UNCOLLIDE ITSELF FROM JUNCTION YET
    public void initThings(Pose2d startPos, StartTile startTile, ActionObjectOld[] actionObjects, int parkPosition){
        enhancedDriver = new EnhancedDriver(hardwareMap);
        enhancedDriver.initPosition(startPos);
        this.actionObjects = new ArrayList<>(Arrays.asList(actionObjects));

        List<ActionObjectOld> newLocations = calculateParking(startTile, this.actionObjects, parkPosition);
        for (ActionObjectOld newLocation : newLocations){
            this.actionObjects.add(newLocation);
        }
    }

    public void initThings(){

    }


    public void reflectMethods(){
        Class EnhancedDriver = enhancedDriver.getClass();
        enhancedDriverMethods = EnhancedDriver.getDeclaredMethods();


    }






    public ArrayList<ActionObjectOld> calculateParking(StartTile startTile, List<ActionObjectOld> actionObjects, int parkPosition){
        //TODO: Everything

        ArrayList<ActionObjectOld> newLocations = new ArrayList<>(0);
        ActionObjectOld lastPosition = actionObjects.get(actionObjects.size()-1);


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
            // will probably just backtrack until no more collision tbh
            // we are going to ignore this problem for now and just make the path never end
            // in collision with a junction



        }

        /**
         * calculates the nearest center of tile
         * TODO: MAKE FUNCTIONAL
         *
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
        */

        /** temporary workaround
         *
         */
        switch(startTile){
            case F2:
            case F5:
                newLocations.add(new ActionObjectOld(new Pose2d(lastX, -12, lastHeading)));
                break;
            case A2:
            case A5:
                newLocations.add(new ActionObjectOld(new Pose2d(lastX, 12, lastHeading)));
        }


        /**
         * first checks to see if it is already in position
         * if so, just adjusts the heading accordingly and returns the locations
         */

        if (Math.abs(newLocations.get(newLocations.size()-1).getPose2d().getX() - parkingGoal.getX()) < 4 && Math.abs(newLocations.get(newLocations.size()-1).getPose2d().getY() - parkingGoal.getY()) < 4) {
            newLocations.add(new ActionObjectOld(parkingGoal));
            return newLocations;
        }


        /**
         * moves onto the "rail line" first after turning to the correct orientation
         */
        Pose2d lastNewPose = newLocations.get(newLocations.size()-1).getPose2d();
        newLocations.add(new ActionObjectOld(new Pose2d(lastNewPose.getX(), lastNewPose.getY(), parkingGoal.getHeading())));
        newLocations.add(new ActionObjectOld(new Pose2d(lastNewPose.getX(), parkingGoal.getY(), parkingGoal.getHeading())));
        newLocations.add(new ActionObjectOld(parkingGoal));


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
             * This code sums the angle between the vectors
             */
            double totalAngle = 0;
            for(int i = 0; i < 4; i++){
                totalAngle+=vector2dList.get(i).angleBetween(vector2dList.get((i+1)%4));
            }
            /**
             * this should be a good approximation of collision
             */
            if (totalAngle > Math.toRadians(355) && totalAngle < Math.toRadians(365)){
                collidingJunctions.add(junction);
            }
        }
        return collidingJunctions;
    }

    private Pose2d[] calculateCollisionSquarePoints(double angle, Pose2d lastPose) {
        angle+=Math.toRadians(45);
        Pose2d[] square = new Pose2d[4];
        for (int i = 0; i < 4; i++){
            angle%=Math.toRadians(360);
            square[i] = new Pose2d(
                    9*Math.sqrt(2)*Math.cos(angle)+lastPose.getX(),
                    9*Math.sqrt(2)*Math.sin(angle)+lastPose.getY());
            angle+=Math.toRadians(90);
        }
        return square;
    }

    public Pose2d calculateTargetPositions(StartTile startTile, int parkPosition) {
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

        //blue offset
        if (startTile == StartTile.A2 || startTile == StartTile.A5) {
            switch (parkPosition) {
                case (1):
                    parkingGoal = new Pose2d(parkingGoal.getX() + 24, parkingGoal.getY());
                case (2):
                    parkingGoal = new Pose2d(parkingGoal.getX() + 24, parkingGoal.getY());
                case (3):
                    break;
            }
        }
        //red offset
        else if (startTile == StartTile.F2 || startTile == StartTile.F5) {
            switch (parkPosition) {
                case (3):
                    parkingGoal = new Pose2d(parkingGoal.getX() + 24, parkingGoal.getY());
                case (2):
                    parkingGoal = new Pose2d(parkingGoal.getX() + 24, parkingGoal.getY());
                case (1):
                    break;
            }
        }

        return parkingGoal;
    }




    public int readAprilTag(){
        detector = new AprilTagDetector(hardwareMap);
        int pos = detector.getPos();
        //detector.endStream();

        //store it in the form of a Pose2D to parkLocation if you would
        return pos;
    }

}
