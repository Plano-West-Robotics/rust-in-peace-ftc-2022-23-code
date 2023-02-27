package org.firstinspires.ftc.teamcode.configs;

import static org.firstinspires.ftc.teamcode.configs.JunctionPoints.generateJunctionPoints;
import static org.firstinspires.ftc.teamcode.configs.ParkingLocations.A2Point1;
import static org.firstinspires.ftc.teamcode.configs.ParkingLocations.A5Point1;
import static org.firstinspires.ftc.teamcode.configs.ParkingLocations.F2Point1;
import static org.firstinspires.ftc.teamcode.configs.ParkingLocations.F5Point1;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.firstinspires.ftc.teamcode.configs.StartingTiles.StartTile;
import org.firstinspires.ftc.teamcode.driveobjs.drivers.EnhancedDriver;
import org.firstinspires.ftc.teamcode.driveobjs.instructables.Instruction;

import java.util.ArrayList;
import java.util.List;

public class ParkingCalculator {
    public static String parkingTrigger = "BEGIN_PARKING_SEQUENCE";

    public static ArrayList<Instruction> calculateParking(String triggerTag, StartTile startTile, Pose2d lastPose, int parkPosition, EnhancedDriver enhancedDriver){
        /**
         * does a quick check to see that there is actually a thing to process here
         */
        if (parkPosition == 0)
            return null;

        ArrayList<Pose2d> newLocations = new ArrayList<>(0);
        parkingTrigger = triggerTag;

        double lastX = lastPose.getX();
        double lastY = lastPose.getY();
        double lastHeading = lastPose.getHeading();

        //gets the actual Pose2d representing the ideal parking location
        Pose2d parkingGoal = calculateTargetPositions(startTile, parkPosition);

        Pose2d[] collisionSquare = calculateCollisionSquarePoints(lastHeading, lastPose);

        List<Pose2d> collisionPoints = checkForCollision(collisionSquare, lastPose);

        if (!collisionPoints.isEmpty()){
            // dashboardTelemetry.addLine("Collision Detected");
            // dashboardTelemetry.update();
            // TODO: Get out of the collision and realign
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
                newLocations.add(new Pose2d(lastX, -12, lastHeading));
                break;
            case A2:
            case A5:
                newLocations.add(new Pose2d(lastX, 12, lastHeading));
        }


        /**
         * first checks to see if it is already in position
         * if so, just adjusts the heading accordingly and returns the locations
         */

        if (Math.abs(newLocations.get(newLocations.size()-1).getX() - parkingGoal.getX()) < 4 && Math.abs(newLocations.get(newLocations.size()-1).getY() - parkingGoal.getY()) < 4) {
            newLocations.add(parkingGoal);

        }


        /**
         * moves onto the "rail line" first after turning to the correct orientation
         */
        else {
            Pose2d lastNewPose = newLocations.get(newLocations.size() - 1);
            //get onto the correct heading first
            newLocations.add(new Pose2d(lastNewPose.getX(), lastNewPose.getY(), parkingGoal.getHeading()));
            //gets to the correct y
            newLocations.add(new Pose2d(lastNewPose.getX(), parkingGoal.getY(), parkingGoal.getHeading()));
            //gets to the correct location
            newLocations.add(parkingGoal);

        }

        return translatePoseToInstruction(enhancedDriver, newLocations);

        //Instruction[] instructions = {new Instruction()}
        //return
    }

    /**
     * packages the poses into instructions
     * @param driver the driver to use
     * @param newLocations the list of poses
     * @return the list of instructions
     */
    private static ArrayList<Instruction> translatePoseToInstruction(EnhancedDriver driver, ArrayList<Pose2d> newLocations){
        ArrayList<Instruction> instructions = new ArrayList<Instruction>();
        for (int i = 0; i < newLocations.size(); i++) {
            int finalI = i;
            instructions.add(driver.makeInstruction(parkingTrigger, parkingTrigger+i, () ->
                    driver.driveTo(newLocations.get(finalI)))
            );
            parkingTrigger = parkingTrigger+i;
        }
        return instructions;
    }

    private static ArrayList<Pose2d> checkForCollision(Pose2d[] square, Pose2d lastPose) {
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

    private static Pose2d[] calculateCollisionSquarePoints(double angle, Pose2d lastPose) {
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

    public static Pose2d calculateTargetPositions(StartTile startTile, int parkPosition) {
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




}
