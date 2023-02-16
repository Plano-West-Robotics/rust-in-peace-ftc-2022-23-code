package org.firstinspires.ftc.teamcode.driveobjs.instructables.instructedAutos;

import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.firstinspires.ftc.teamcode.driveobjs.instructables.InstructionRunner;

public class InstructableA2 extends InstructableBase{
    InstructionRunner runner;
    Pose2d startPose = new Pose2d(0, 0, 0);

    public void runOpMode() throws InterruptedException {
        runner = new InstructionRunner(hardwareMap, startPose, "START");




        waitForTag();




        while (opModeIsActive() && !isStopRequested()) {
            runner.run();
            telemetry.update();



        }




    }
}
