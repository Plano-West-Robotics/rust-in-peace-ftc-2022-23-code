package org.firstinspires.ftc.teamcode.enhancedautos;

import static org.firstinspires.ftc.teamcode.configs.HardwareNames.spoolMotorName;

import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.driveobjs.ClawDriver;
import org.firstinspires.ftc.teamcode.driveobjs.aprilTag.AprilTagDetector;
import org.firstinspires.ftc.teamcode.roadRunner.drive.SampleMecanumDrive;

@Autonomous
public class fallbackAutoDev extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        int parkPosition = 0;
        AprilTagDetector detector = null;

        DcMotor spoolMotor = hardwareMap.get(DcMotor.class, spoolMotorName);
        spoolMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //spoolMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        spoolMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        spoolMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        while (!isStarted() && !isStopRequested()) {
            detector = new AprilTagDetector(hardwareMap);
            while (!isStarted() && !isStopRequested()) {
                parkPosition = detector.getPos();
                telemetry.addLine(String.format("\nDetected tag ID=%d", parkPosition));
                telemetry.update();
                sleep(50);
            }

        }

        SampleMecanumDrive driver = new SampleMecanumDrive(hardwareMap);
        Trajectory setPosition = null;

        if (parkPosition < 1){
            long startTime = System.currentTimeMillis();
            for (int i = 0; i < 50; i++ ) {
                parkPosition = detector.getPos();
                telemetry.addLine(String.format("\nDetected tag ID=%d", parkPosition));
                telemetry.update();
                sleep(50);
            }
        }

        detector.endStream();
        if (parkPosition < 1){
            parkPosition = 1;
        }

        ClawDriver clawDriver = new ClawDriver(hardwareMap);
        clawDriver.close();
        sleep(1000);

        spoolMotor.setTargetPosition(500);
        spoolMotor.setPower(0.6);
        while (spoolMotor.isBusy()) {}
        spoolMotor.setPower(0);

        driver.followTrajectory(driver.trajectoryBuilder(driver.getPoseEstimate()).forward(2).build());
        driver.followTrajectory(driver.trajectoryBuilder(driver.getPoseEstimate()).strafeLeft(23).build());
        driver.followTrajectory(driver.trajectoryBuilder(driver.getPoseEstimate()).forward(48).build());
        driver.followTrajectory(driver.trajectoryBuilder(driver.getPoseEstimate()).strafeRight(13.5).build());

        spoolMotor.setTargetPosition(3900);
        spoolMotor.setPower(0.6);
        while (spoolMotor.isBusy()) {}
        spoolMotor.setPower(0.01);

        driver.followTrajectory(driver.trajectoryBuilder(driver.getPoseEstimate()).forward(8).build());

        sleep(2000);

        spoolMotor.setTargetPosition(spoolMotor.getCurrentPosition()-400);
        spoolMotor.setPower(-0.6);
        while (spoolMotor.isBusy()) {}
        spoolMotor.setPower(0.01);

        clawDriver.open();

        sleep(1000);

        driver.followTrajectory(driver.trajectoryBuilder(driver.getPoseEstimate()).forward(-8).build());

        switch (parkPosition) {
            case 1:
                setPosition = driver.trajectoryBuilder(driver.getPoseEstimate())
                        .strafeLeft(13)
                        .build();
                break;
            case 2:
                setPosition = driver.trajectoryBuilder(driver.getPoseEstimate())
                        .strafeRight(13)
                        .build();
                break;
            case 3:
                setPosition = driver.trajectoryBuilder(driver.getPoseEstimate())
                        .strafeRight(34)
                        .build();
                break;

        }

        driver.followTrajectory(setPosition);

        spoolMotor.setTargetPosition(spoolMotor.getCurrentPosition()-3000);
        spoolMotor.setPower(-0.6);
        while (spoolMotor.isBusy()) {}
        spoolMotor.setPower(0);

        while(opModeIsActive()){
//            clawDriver.close();
        }
    }


}