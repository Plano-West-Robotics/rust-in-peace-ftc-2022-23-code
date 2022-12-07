package org.firstinspires.ftc.teamcode.troubleshooting;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.driveobjs.LinearSlideDriver;

@Autonomous
public class LinearSlideTuner extends LinearOpMode {
    LinearSlideDriver linearSlideDriver;
    int target = 0;
    public static double speedMultiplier = 1;

    public void runOpMode(){
        linearSlideDriver = new LinearSlideDriver(hardwareMap);



        waitForStart();

        while (isStarted() && !isStopRequested()){
            if (Math.abs(gamepad1.right_stick_y) > 0){
                target += gamepad1.right_stick_y * speedMultiplier;
            }

            telemetry.addData("Target Height", linearSlideDriver.getTargetEncoderValue());


            if (gamepad1.a){
                target = LinearSlideDriver.height1;
            }

            if (gamepad1.b){
                target = LinearSlideDriver.height2;
            }

            if (gamepad1.x){
                target = LinearSlideDriver.height3;
            }

            if (gamepad1.y){
                target = LinearSlideDriver.height4;
            }


            int[] slidePIDOutput = linearSlideDriver.run();
            telemetry.addData("Slide Target", slidePIDOutput[0]);
            telemetry.addData("Slide Current", slidePIDOutput[1]);
            telemetry.addData("Slide Error", slidePIDOutput[2]);

            if (gamepad1.start){
                linearSlideDriver = new LinearSlideDriver(hardwareMap);
            }

            telemetry.update();

        }
    }

}
