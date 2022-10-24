package org.firstinspires.ftc.teamcode.driveobjs;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

import java.util.List;

public class EnhancedDriver extends SampleMecanumDrive{

    private HardwareMap hardwareMap;
    private boolean isFirstAction = true;
    private TelemetryPacket packet = new TelemetryPacket();
    private FtcDashboard dashboard = FtcDashboard.getInstance();


    public EnhancedDriver(HardwareMap hardwareMap){
        super(hardwareMap);
        this.hardwareMap = hardwareMap;
        init();
    }

    public void run(List<ActionObject> actionObjects){
        setPoseEstimate(actionObjects.remove(0).getPose2d());
        for(ActionObject actionObject : actionObjects) {
            //checks for location change, and then moves to that location
            Pose2d newPose = actionObject.getPose2d();
            Pose2d lastPose = getPoseEstimate();
            if (!(newPose.getX() == lastPose.getX() && newPose.getY() == lastPose.getY())) {
                Trajectory traj = trajectoryBuilder(lastPose)
                        .lineToLinearHeading(newPose)
                        .build();
                followTrajectory(traj);
            }
            executeAction(actionObject.getMethodID());
        }
    }


    /*
     public void act(ActionObject args) {
         if (isFirstAction){
             lastPose = args.getPose2d();
             setPoseEstimate(lastPose);
             isFirstAction = false;
         }
         else {
             Pose2d newPose = args.getPose2d();
             //checks to make sure that the poses are different, avoids no trajectory error
             if (!(newPose.getX() == lastPose.getX() && newPose.getY() == lastPose.getY())) {
                 Trajectory traj = trajectoryBuilder(lastPose)
                         .lineToLinearHeading(newPose)
                         .build();
                 new Pose2d();
                 followTrajectory(traj);
                 lastPose = newPose;
             }
         }
         if (args.getMethodID() != 0) {
             try {
                 executeAction(args.getMethodID());
             } catch (IndexOutOfBoundsException e) {
                 packet.put("Invalid Method ID", args.getMethodID() + " is not a valid ID");
                 flushTelemetry();
             }
         }
    }*/

    public void executeAction(int id) throws IndexOutOfBoundsException{
        int subIndex = id%10;
        switch(id/10){
        }

    }


    //sleeps for a certain amount of milliseconds
    private void sleep(long milli){
        try{
            Thread.sleep(milli);
        }
        catch (InterruptedException e){
            e.printStackTrace();
        }
    }



    //sends off the telemetry values
    private void flushTelemetry(){
        dashboard.sendTelemetryPacket(packet);
    }

    //initializes the parts
    public void init(){
        //put parts here
    }


}