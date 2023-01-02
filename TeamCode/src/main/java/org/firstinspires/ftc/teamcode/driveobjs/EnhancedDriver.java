package org.firstinspires.ftc.teamcode.driveobjs;


import static org.firstinspires.ftc.teamcode.configs.HardwareNames.spoolMotorName;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.configs.PoseStorage;
import org.firstinspires.ftc.teamcode.roadRunner.drive.SampleMecanumDrive;

import java.util.List;

public class EnhancedDriver extends SampleMecanumDrive{
    private DcMotor frontLeftMotor, frontRightMotor, backLeftMotor, backRightMotor;
    private DcMotor spoolMotor;
    private CRServo grabServo;

    private ClawDriver clawDriver;


    private HardwareMap hardwareMap;
    private boolean isFirstAction = true;
    private TelemetryPacket packet = new TelemetryPacket();
    private FtcDashboard dashboard = FtcDashboard.getInstance();
    Telemetry dashboardTelemetry = dashboard.getTelemetry();

    private enum ClawState {open, close}
    private ClawState  clawState = ClawState.close;

    public EnhancedDriver(HardwareMap hardwareMap){
        super(hardwareMap);
        this.hardwareMap = hardwareMap;
        init();
    }

    public void initPosition(Pose2d startingPos){
        setPoseEstimate(startingPos);

    }
    public void moveConeOutOfWay(){
        clawDriver.close();
        sleep(1000);
        spoolMotor.setPower(1);
        sleep(1000);
        spoolMotor.setPower(0);
    }

    public void run(List<ActionObject> actionObjects){
        //setPoseEstimate(actionObjects.remove(0).getPose2d());
        for(ActionObject actionObject : actionObjects) {
            //checks for location change, and then moves to that location
            Pose2d newPose = actionObject.getPose2d();
            Pose2d lastPose = getPoseEstimate();

            dashboardTelemetry.addData("x", newPose.getX());
            dashboardTelemetry.addData("y", newPose.getY());
            dashboardTelemetry.addData("heading", newPose.getHeading());
            dashboardTelemetry.update();

            //ensures that there is no empty path exception
            if (!(newPose.getX() == lastPose.getX() && newPose.getY() == lastPose.getY())) {
                Trajectory traj = trajectoryBuilder(lastPose)
                        .lineToLinearHeading(newPose)
                        .build();
                followTrajectory(traj);
            }
            //if there is an empty path, checks to see if there is an angle change, if so, executes that angle change
            else if (newPose.getHeading() != lastPose.getHeading()){
                Trajectory traj = trajectoryBuilder(lastPose)
                        .lineToLinearHeading(new Pose2d(lastPose.getX(), lastPose.getY()-0.01, newPose.getHeading()))
                        .lineToLinearHeading(newPose)
                        .build();
                followTrajectory(traj);

            }
            /**
             * adds the current pose to the pose storage class for transfer across OpModes
             */
            PoseStorage.currentPose = getPoseEstimate();

            /**
             * executes the action specified by the ID
             * IMPORTANT: THIS EXECUTES AFTER THE MOVEMENT
             */
            executeAction(actionObject.getMethodID());
            switch(clawState){
                case open:
                    clawDriver.open();
                    break;
                case close:
                    clawDriver.close();
                    break;
            }

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
            case 0:
                break;
            case 1:
                moveArm(subIndex);
                break;
            case 2:
                turnGrabber(subIndex);
                break;
            default:
                throw new IndexOutOfBoundsException("METHOD ID DOES NOT EXIST");
        }


    }

    private void turnGrabber(int subIndex) {
        switch(subIndex){
            case 0:
                clawDriver.close();
                clawState = ClawState.close;
                sleep(1000);
                break;
            case 1:
                clawDriver.open();
                clawState = clawState.open;
                sleep(1000);
                break;
        }
    }

    private void moveArm(int subIndex) {
        switch(subIndex){
            case 0:
                spoolMotor.setPower(1);
                sleep(100);
                break;
            case 2:
                spoolMotor.setPower(-1);
                sleep(1000);
            case 3:
                spoolMotor.setPower(1);
                sleep(100);
                break;




            /**
             * TODO: Fix encoder issue and implement this
             *
            case 0:
                spoolMotor.setTargetPosition(ArmPosition1EncoderCount);
                spoolMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                spoolMotor.setPower(0.5);
                break;
            case 1:
                spoolMotor.setTargetPosition(ArmPosition2EncoderCount);
                spoolMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                spoolMotor.setPower(0.5);
                break;
            case 2:
                spoolMotor.setTargetPosition(ArmPosition3EncoderCount);
                spoolMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                spoolMotor.setPower(0.5);
                break;
            case 3:
                spoolMotor.setTargetPosition(ArmPosition4EncoderCount);
                spoolMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                spoolMotor.setPower(0.5);
                break;
             */
        }
        //while(spoolMotor.isBusy()){sleep(1);}
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

        clawDriver = new ClawDriver(hardwareMap);

        //grabServo = hardwareMap.get(CRServo.class, grabServo1Name);
        //grabServo.resetDeviceConfigurationForOpMode();

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
