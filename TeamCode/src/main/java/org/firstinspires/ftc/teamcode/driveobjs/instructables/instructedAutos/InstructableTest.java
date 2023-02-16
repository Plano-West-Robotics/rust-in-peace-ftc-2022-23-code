package org.firstinspires.ftc.teamcode.driveobjs.instructables.instructedAutos;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;

import org.firstinspires.ftc.teamcode.driveobjs.drivers.ClawDriver;
import org.firstinspires.ftc.teamcode.driveobjs.instructables.InstructionRunner;

public class InstructableTest extends InstructableBase {
    InstructionRunner runner;
    Pose2d startPose = new Pose2d(0, 0, 0);

    public void runOpMode() throws InterruptedException {
        runner = new InstructionRunner(hardwareMap, startPose, "START");

        runner.addInstruction(claw.makeInstruction("START", () -> {
            claw.setClawState(ClawDriver.ClawState.OPEN);
        }));

        Trajectory Traj1 = driver.trajectoryBuilder(startPose)
                .lineToConstantHeading(new Vector2d(0, 10))
                .build();
        runner.addInstruction(driver.makeInstruction("START", "TRAJ1", () -> driver.followTrajectoryAsync(Traj1)));

        Trajectory Traj2 = driver.trajectoryBuilder(Traj1.end())
                .strafeRight(10)
                .build();
        runner.addInstruction(driver.makeInstruction("TRAJ1", "TRAJ2", Traj2));

        runner.addInstruction(claw.makeInstruction("TRAJ2", () -> claw.open()));



        waitForTag();
        while (opModeIsActive() && !isStopRequested()) {
            runner.run();
            telemetry.update();
            driver.update();


        }




    }
}
