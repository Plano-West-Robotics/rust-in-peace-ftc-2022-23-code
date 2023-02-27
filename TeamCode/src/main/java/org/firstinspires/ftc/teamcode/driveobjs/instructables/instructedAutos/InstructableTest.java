package org.firstinspires.ftc.teamcode.driveobjs.instructables.instructedAutos;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.driveobjs.aprilTag.AprilTagDetector;
import org.firstinspires.ftc.teamcode.driveobjs.drivers.ClawDriver;
import org.firstinspires.ftc.teamcode.driveobjs.drivers.EnhancedDriver;
import org.firstinspires.ftc.teamcode.driveobjs.drivers.LinearSlideDriver;
import org.firstinspires.ftc.teamcode.driveobjs.instructables.InstructionRunner;

/**
 * this should close the claw, move up by ten, strafe right, then open the claw
 */
@Autonomous
public class InstructableTest extends InstructableBase {
    InstructionRunner runner;
    ClawDriver claw;
    EnhancedDriver driver;
    LinearSlideDriver slide;
    Pose2d startPose = new Pose2d(0, 0, 0);

    public void runOpMode() throws InterruptedException {
        claw = new ClawDriver(hardwareMap);
        driver = new EnhancedDriver(hardwareMap);
        slide = new LinearSlideDriver(hardwareMap);
        runner = new InstructionRunner(hardwareMap, startPose, "START", telemetry, claw, driver, slide);

        runner.addInstruction(claw.makeInstruction("START", () -> {
            claw.setClawState(ClawDriver.ClawState.CLOSED);
        }));

        Trajectory Traj1 = driver.trajectoryBuilder(startPose)
                .forward(72)
                .build();
        runner.addInstruction(driver.asyncPathFollowInstruction("START", "TRAJ1", Traj1));

        Trajectory Traj2 = driver.trajectoryBuilder(Traj1.end())
                .strafeRight(10)
                .build();
        runner.addInstruction(driver.asyncPathFollowInstruction("TRAJ1", "TRAJ2", Traj2));

        runner.addInstruction(claw.makeInstruction("TRAJ2", () -> claw.open()));



        waitForTag(new AprilTagDetector(hardwareMap));

        telemetry.addLine("Tags" + runner.getTags());
        telemetry.update();
        sleep(1000);



        while (opModeIsActive() && !isStopRequested()) {
            String output = runner.run();
            telemetry.addLine("Tags: " + output);
            telemetry.update();
            driver.update();
        }




    }
}
