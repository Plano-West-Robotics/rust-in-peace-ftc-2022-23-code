package org.firstinspires.ftc.teamcode.instructedAutos;

import static org.firstinspires.ftc.teamcode.roadRunner.drive.DriveConstants.MAX_ANG_VEL;
import static org.firstinspires.ftc.teamcode.roadRunner.drive.DriveConstants.TRACK_WIDTH;
import static org.firstinspires.ftc.teamcode.roadRunner.drive.SampleMecanumDrive.getAccelerationConstraint;
import static org.firstinspires.ftc.teamcode.roadRunner.drive.SampleMecanumDrive.getVelocityConstraint;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.configs.ParkingCalculator;
import org.firstinspires.ftc.teamcode.configs.StartingTiles;
import org.firstinspires.ftc.teamcode.driveobjs.aprilTag.AprilTagDetector;
import org.firstinspires.ftc.teamcode.driveobjs.drivers.ClawDriver;
import org.firstinspires.ftc.teamcode.driveobjs.drivers.EnhancedDriver;
import org.firstinspires.ftc.teamcode.driveobjs.drivers.LinearSlideDriver;
import org.firstinspires.ftc.teamcode.driveobjs.drivers.TimerDriver;
import org.firstinspires.ftc.teamcode.driveobjs.instructables.Instruction;
import org.firstinspires.ftc.teamcode.driveobjs.instructables.InstructionRunner;

import java.util.ArrayList;
import java.util.Objects;

@Autonomous
public class InstructableA5 extends InstructableBase{
    InstructionRunner runner;
    ClawDriver claw;
    EnhancedDriver driver;
    LinearSlideDriver slide;

    Pose2d startPose = new Pose2d(38, 63, Math.toRadians(-90));

    public void runOpMode() throws InterruptedException {
        claw = new ClawDriver(hardwareMap);
        driver = new EnhancedDriver(hardwareMap);
        slide = new LinearSlideDriver(hardwareMap, telemetry);
        runner = new InstructionRunner(hardwareMap, startPose, "START", telemetry, claw, driver, slide);

        driver.setPoseEstimate(startPose);


        TrajectoryVelocityConstraint VEL_CONSTRAINT = getVelocityConstraint(30, MAX_ANG_VEL, TRACK_WIDTH);
        TrajectoryAccelerationConstraint ACCEL_CONSTRAINT = getAccelerationConstraint(30);

        Trajectory traj1 = driver.trajectoryBuilder(startPose)
                .splineToConstantHeading(new Vector2d(30, 61), Math.toRadians(-180))
                .lineToConstantHeading(new Vector2d(18, 61), VEL_CONSTRAINT, ACCEL_CONSTRAINT)
                .splineToConstantHeading(new Vector2d(12, 50), Math.toRadians(-90), VEL_CONSTRAINT, ACCEL_CONSTRAINT)
                .lineToSplineHeading(new Pose2d(12, 30, Math.toRadians(-180)), VEL_CONSTRAINT, ACCEL_CONSTRAINT)
                .splineToConstantHeading(new Vector2d(9, 23), Math.toRadians(-180), VEL_CONSTRAINT, ACCEL_CONSTRAINT)
                .build();


        runner.addInstruction(driver.asyncPathFollowInstruction("START", "TRAJ1", traj1));



        runner.addInstruction(claw.closeInstruction("START"));
        runner.addInstruction(TimerDriver.waitInstruction("START", "STARTSLIDE1", 2500));

        runner.addInstruction(slide.makeInstruction("STARTSLIDE1", "SLIDE1", () -> {slide.setTargetHigh();}));

        runner.addInstruction(TimerDriver.waitInstruction("SLIDE1", "FALLWAIT", 2000));
        runner.addInstruction(claw.openInstruction("FALLWAIT"));
        runner.addInstruction(TimerDriver.waitInstruction("FALLWAIT", "FINISHOPEN", 1000));

        Trajectory traj2 = driver.trajectoryBuilder(traj1.end())
                .splineToConstantHeading(new Vector2d(12, 24), Math.toRadians(-90))
                .lineToConstantHeading(new Vector2d(12, 9))
                .build();
        runner.addInstruction(driver.asyncPathFollowInstruction("FINISHOPEN", "START_PARKING_SEQUENCE", traj2));



        int parkPosition = 0;
        AprilTagDetector detector;
        detector = new AprilTagDetector(hardwareMap);
        ArrayList<Instruction> parkingInstructions = null;
        while (!isStarted() && !isStopRequested()) {
            parkPosition = waitForTag(detector);
            telemetry.addLine(String.format("\nDetected tag ID=%d", parkPosition));
            if (!Objects.isNull(parkingInstructions)){
                for (Instruction i : parkingInstructions)  {
                    telemetry.addLine(i.toString());
                }
            }
            //telemetry.addData(parkingInstructions.toString());
            telemetry.update();
            parkingInstructions = ParkingCalculator.calculateParking("START_PARKING_SEQUENCE", StartingTiles.StartTile.A5, traj2.end(), parkPosition, driver);
        }
        if (parkPosition < 1){
            parkPosition = 1;
            parkingInstructions = ParkingCalculator.calculateParking("START_PARKING_SEQUENCE", StartingTiles.StartTile.A5, traj2.end(), parkPosition, driver);
        }


        for (Instruction i : parkingInstructions){
            runner.addInstruction(i);
        }





        while (opModeIsActive() && !isStopRequested()) {
            runner.run();
            telemetry.update();



        }




    }
}
