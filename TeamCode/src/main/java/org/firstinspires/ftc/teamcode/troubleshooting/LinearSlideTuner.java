package org.firstinspires.ftc.teamcode.troubleshooting;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.driveobjs.LinearSlideDriver;

@Autonomous
@Config
public class LinearSlideTuner extends LinearOpMode {
    LinearSlideDriver linearSlideDriver;
    int target = 0;
    public static double speedMultiplier = 1;
    public static int max = 500;
    public static int min = 100;

    public void runOpMode(){
        linearSlideDriver = new LinearSlideDriver(hardwareMap);
        boolean manualMode = true;
        long startTime = System.currentTimeMillis();

        waitForStart();

        while (isStarted() && !isStopRequested()){
            if (gamepad1.start){
                manualMode = !manualMode;
            }
            if (!manualMode){
                if (System.currentTimeMillis() - startTime < 5000) {
                    target = max;
                }
                else if (System.currentTimeMillis() - startTime < 10000) {
                    target = min;
                }
                else startTime = System.currentTimeMillis();
            }
            else{

                if (Math.abs(gamepad1.right_stick_y) > 0) {
                    target += gamepad1.right_stick_y * speedMultiplier;
                }

                telemetry.addData("Target Height", linearSlideDriver.getTargetEncoderValue());


                if (gamepad1.a) {
                    target = LinearSlideDriver.height1;
                }

                if (gamepad1.b) {
                    target = LinearSlideDriver.height2;
                }

                if (gamepad1.x) {
                    target = LinearSlideDriver.height3;
                }

                if (gamepad1.y) {
                    target = LinearSlideDriver.height4;
                }




                if (gamepad1.start) {
                    linearSlideDriver = new LinearSlideDriver(hardwareMap);
                }
            }

            linearSlideDriver.setTarget(target);
            int[] slidePIDOutput = linearSlideDriver.run();
            telemetry.addData("Slide Target", slidePIDOutput[0]);
            telemetry.addData("Slide Current", slidePIDOutput[1]);
            telemetry.addData("Slide Error", slidePIDOutput[2]);
            telemetry.update();

        }
    }

}
