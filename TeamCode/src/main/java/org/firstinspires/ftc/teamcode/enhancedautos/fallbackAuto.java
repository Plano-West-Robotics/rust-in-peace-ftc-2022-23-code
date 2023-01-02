package org.firstinspires.ftc.teamcode.enhancedautos;

import static org.firstinspires.ftc.teamcode.configs.HardwareNames.spoolMotorName;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.driveobjs.ClawDriver;
import org.firstinspires.ftc.teamcode.driveobjs.aprilTag.AprilTagDetector;
import org.firstinspires.ftc.teamcode.roadRunner.drive.SampleMecanumDrive;

@Autonomous
public class fallbackAuto extends LinearOpMode {
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

        spoolMotor.setPower(1);
        sleep(1000);
        spoolMotor.setPower(0);

        switch (parkPosition) {
            case 1:
                setPosition = driver.trajectoryBuilder(new Pose2d())
                        .strafeLeft(23)
                        .build();
                break;
            case 2:
                setPosition = driver.trajectoryBuilder(new Pose2d())
                        .strafeRight(2)
                        .build();
                break;
            case 3:
                setPosition = driver.trajectoryBuilder(new Pose2d())
                        .strafeRight(25)
                        .build();
                break;

        }
        driver.followTrajectory(driver.trajectoryBuilder(driver.getPoseEstimate()).forward(2).build());
        driver.followTrajectory(setPosition);
        driver.followTrajectory(driver.trajectoryBuilder(driver.getPoseEstimate()).forward(30).build());
        while(opModeIsActive()){
            clawDriver.close();
        }
    }


}
