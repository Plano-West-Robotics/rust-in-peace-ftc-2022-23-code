package org.firstinspires.ftc.teamcode.driveobjs.instructables.instructedAutos;

import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.firstinspires.ftc.teamcode.driveobjs.drivers.ClawDriver;
import org.firstinspires.ftc.teamcode.driveobjs.instructables.InstructionRunner;

public class InstructableTest extends InstructableBase {
    InstructionRunner runner;
    Pose2d startPose = new Pose2d(0, 0, 0);

    public void runOpMode() throws InterruptedException {
        runner = new InstructionRunner(hardwareMap, startPose, "START");

        waitForTag();



        claw.makeInstruction("START", () -> {
            claw.setClawState(ClawDriver.ClawState.OPEN);
        });





        while (opModeIsActive() && !isStopRequested()) {



            telemetry.update();
        }




    }
}
