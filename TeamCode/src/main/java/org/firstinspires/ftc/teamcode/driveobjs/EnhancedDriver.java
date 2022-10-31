package org.firstinspires.ftc.teamcode.driveobjs;


import static org.firstinspires.ftc.teamcode.configs.HardwareNames.*;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.roadRunner.drive.SampleMecanumDrive;



import java.util.List;

public class EnhancedDriver extends SampleMecanumDrive{
    private DcMotor frontLeftMotor, frontRightMotor, backLeftMotor, backRightMotor;
    private DcMotor spoolMotor;
    private CRServo grabServo;



    private HardwareMap hardwareMap;
    private boolean isFirstAction = true;
    private TelemetryPacket packet = new TelemetryPacket();
    private FtcDashboard dashboard = FtcDashboard.getInstance();


    public EnhancedDriver(HardwareMap hardwareMap){
        super(hardwareMap);
        this.hardwareMap = hardwareMap;
        init();
    }

    public void initPosition(Pose2d startingPos){
        setPoseEstimate(startingPos);
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
        /*
        frontLeftMotor = hardwareMap.get(DcMotor.class, frontLeftMotorName);
        frontRightMotor = hardwareMap.get(DcMotor.class, frontRightMotorName);
        backLeftMotor = hardwareMap.get(DcMotor.class, backLeftMotorName);
        backRightMotor = hardwareMap.get(DcMotor.class, backRightMotorName);
        */



        spoolMotor = hardwareMap.get(DcMotor.class, spoolMotorName);
        spoolMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        spoolMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        spoolMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        grabServo = hardwareMap.get(CRServo.class, grabServoName);
        grabServo.resetDeviceConfigurationForOpMode();

        /*

        grabServo = hardwareMap.get(Servo.class, "grabServo");
        carouselMotor1 = hardwareMap.get(DcMotor.class, "spinnyBoyOne");
        carouselMotor2 = hardwareMap.get(DcMotor.class, "spinnyBoyTwo");
        armOne = hardwareMap.get(DcMotor.class, "armOne");
        armTwo = hardwareMap.get(DcMotor.class, "armTwo");
        armOne.setDirection(DcMotorSimple.Direction.REVERSE);
        armOne.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armOne.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armTwo.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armTwo.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armOne.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armTwo.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

         */
           }



}
