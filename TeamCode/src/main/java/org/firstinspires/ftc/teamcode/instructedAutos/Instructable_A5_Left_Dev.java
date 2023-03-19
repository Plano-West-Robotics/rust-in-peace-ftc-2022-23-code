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

import org.firstinspires.ftc.teamcode.configs.ArmPosStorage;
import org.firstinspires.ftc.teamcode.configs.ParkingCalculator;
import org.firstinspires.ftc.teamcode.configs.StartingTiles;
import org.firstinspires.ftc.teamcode.driveobjs.aprilTag.AprilTagDetector;
import org.firstinspires.ftc.teamcode.driveobjs.drivers.ClawDriver;
import org.firstinspires.ftc.teamcode.driveobjs.drivers.EnhancedDriver;
import org.firstinspires.ftc.teamcode.driveobjs.drivers.LinearSlideDriver;
import org.firstinspires.ftc.teamcode.driveobjs.drivers.TimerDriver;
import org.firstinspires.ftc.teamcode.driveobjs.instructables.Instruction;
import org.firstinspires.ftc.teamcode.driveobjs.instructables.InstructionRunner;
import org.firstinspires.ftc.teamcode.roadRunner.trajectorysequence.TrajectorySequence;

import java.util.ArrayList;
import java.util.Objects;

@Autonomous
public class Instructable_A5_Left_Dev extends InstructableBase{
    InstructionRunner runner;
    ClawDriver claw;
    EnhancedDriver driver;
    LinearSlideDriver slide;

    Pose2d startPose = new Pose2d(38, 62, Math.toRadians(-90));

    public void runOpMode() throws InterruptedException {
        claw = new ClawDriver(hardwareMap);
        driver = new EnhancedDriver(hardwareMap);
        slide = new LinearSlideDriver(hardwareMap, telemetry);
        runner = new InstructionRunner(hardwareMap, startPose, "START", telemetry, claw, driver, slide);

        driver.setPoseEstimate(startPose);


        TrajectoryVelocityConstraint VEL_CONSTRAINT = getVelocityConstraint(30, MAX_ANG_VEL, TRACK_WIDTH);
        TrajectoryAccelerationConstraint ACCEL_CONSTRAINT = getAccelerationConstraint(30);

        Trajectory traj1 = driver.trajectoryBuilder(startPose)
                .splineToConstantHeading(new Vector2d(30, 60), Math.toRadians(-180))
                .lineToConstantHeading(new Vector2d(18, 60), VEL_CONSTRAINT, ACCEL_CONSTRAINT)

                .splineToConstantHeading(new Vector2d(12, 50), Math.toRadians(-90), VEL_CONSTRAINT, ACCEL_CONSTRAINT)
                .lineToConstantHeading(new Vector2d(12, 20), VEL_CONSTRAINT, ACCEL_CONSTRAINT)
                .splineToConstantHeading(new Vector2d(18, 14), Math.toRadians(0), getVelocityConstraint(20, MAX_ANG_VEL, TRACK_WIDTH), ACCEL_CONSTRAINT)
                //drop location
                .splineToConstantHeading(new Vector2d(23, 8.5), Math.toRadians(-90), VEL_CONSTRAINT, ACCEL_CONSTRAINT)

                .build();


        runner.addInstruction(driver.asyncPathFollowInstruction("START", "TRAJ_1", traj1));

        runner.addInstruction(claw.closeInstruction("START"));

        runner.addInstruction(TimerDriver.waitInstruction("START", "START_SLIDE", 1600));
        runner.addInstruction(slide.makeInstruction("START", "SLIDE_1", ()->{slide.setTargetHigh();}));

        runner.addInstruction(TimerDriver.waitInstruction("SLIDE_1", "FALL_WAIT", 100, "TRAJ_1"));
        runner.addInstruction(claw.openInstruction("FALL_WAIT", "FINISH_OPEN"));
        /**
         * goes to grab a new cone
         */

        final double FINAL_Y = 11;


        TrajectorySequence traj2 = driver.trajectorySequenceBuilder(traj1.end())
                .splineToConstantHeading(new Vector2d(24, FINAL_Y), 0)
                .lineToConstantHeading(new Vector2d(36, FINAL_Y))
                .turn(Math.toRadians(90))
                .lineToConstantHeading(new Vector2d(63, FINAL_Y))
                .build();

        runner.addInstruction(driver.asyncPathFollowInstruction("FINISH_OPEN", "TRAJ_2", traj2));



        runner.addInstruction(TimerDriver.waitInstruction("FINISH_OPEN", "SLIDE_LOWER",1500));

        runner.addInstruction(slide.makeInstruction("SLIDE_LOWER", "SLIDE_LOWERED", ()->{
            slide.setTargetPosition(ArmPosStorage.stackArmPoses[0]);
        }));

        runner.addInstruction(claw.closeInstruction("TRAJ_2", "GRAB_STACK_1"));

        runner.addInstruction(slide.makeInstruction("GRAB_STACK_1", "SLIDE_MOVED", ()->
                slide.setTargetPosition(slide.getCurrentEncoderValue()-500)));



        /**
         * after picking up a new cone
         */
        TrajectorySequence traj3 = driver.trajectorySequenceBuilder(traj2.end())
                .lineToConstantHeading(new Vector2d(36, FINAL_Y))
                .turn(Math.toRadians(-90))
                .lineToConstantHeading(new Vector2d(30, FINAL_Y))
                //drop location
                .splineToConstantHeading(new Vector2d(25, FINAL_Y-4.5), Math.toRadians(-90))
                .build();

        runner.addInstruction(driver.asyncPathFollowInstruction("SLIDE_MOVED", "TRAJ_3", traj3));

        runner.addInstruction(TimerDriver.waitInstruction("SLIDE_MOVED", "START_SLIDE_HIGH_1", 1200));

        runner.addInstruction(slide.makeInstruction("START_SLIDE_HIGH_1", "SLIDE_HIGH_1", ()->{slide.setTargetHigh();}));

        runner.addInstruction(TimerDriver.waitInstruction("SLIDE_HIGH_1", "FALL_WAIT_2", 100, "TRAJ_3"));
        runner.addInstruction(claw.openInstruction("FALL_WAIT_2", "CLAW_FINISH_1"));



        /**
         * Let's do the 1+2 now
         */

        final double FINAL_Y_2 = 11;

        TrajectorySequence traj4 = driver.trajectorySequenceBuilder(traj3.end())
                .splineToConstantHeading(new Vector2d(30, FINAL_Y_2), 0)
                .turn(Math.toRadians(90))
                .lineToConstantHeading(new Vector2d(64, FINAL_Y_2))
                .build();

        runner.add(driver.asyncPathFollowInstruction("CLAW_FINISH_1", "TRAJ_4", traj4));



        runner.addInstruction(TimerDriver.waitInstruction("CLAW_FINISH_1", "SLIDE_LOWER_2",1500));

        runner.addInstruction(slide.makeInstruction("SLIDE_LOWER_2", "SLIDE_LOWERED_2", ()->{
            slide.setTargetPosition(ArmPosStorage.stackArmPoses[1]);
        }));

        runner.addInstruction(claw.closeInstruction("TRAJ_4", "GRAB_STACK_2"));

        runner.addInstruction(slide.makeInstruction("GRAB_STACK_2", "SLIDE_MOVED_2", ()->
                slide.setTargetPosition(slide.getCurrentEncoderValue()-500)));


        /**
         * after picking up a new cone * 2
         */
        TrajectorySequence traj5 = driver.trajectorySequenceBuilder(traj4.end())
                .lineToConstantHeading(new Vector2d(36, FINAL_Y_2))
                .turn(Math.toRadians(-90))
                .lineToConstantHeading(new Vector2d(30, FINAL_Y_2))
                //drop location
                .splineToConstantHeading(new Vector2d(26, FINAL_Y_2-4.5), Math.toRadians(-90))
                .build();

        runner.addInstruction(driver.asyncPathFollowInstruction("SLIDE_MOVED_2", "TRAJ_5", traj5));

        runner.addInstruction(TimerDriver.waitInstruction("SLIDE_MOVED_2", "START_SLIDE_HIGH_2", 1200));

        runner.addInstruction(slide.makeInstruction("START_SLIDE_HIGH_2", "SLIDE_HIGH_2", ()->{slide.setTargetHigh();}));
        runner.addInstruction(TimerDriver.waitInstruction("SLIDE_HIGH_2", "FALL_WAIT_3", 100, "TRAJ_5"));
        runner.addInstruction(claw.openInstruction("FALL_WAIT_3", "CLAW_FINISH_2"));




        Trajectory trajFinal = driver.trajectoryBuilder(traj3.end())
                .lineToConstantHeading(new Vector2d(26, FINAL_Y_2))
                .build();

        runner.addInstruction(driver.asyncPathFollowInstruction("CLAW_FINISH_2", "START_PARKING_SEQUENCE", trajFinal));
        runner.addInstruction(slide.makeInstruction("START_PARKING_SEQUENCE", "END_SLIDE", ()->{slide.setTargetLow();}));

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
            parkingInstructions = ParkingCalculator.calculateParking("START_PARKING_SEQUENCE", StartingTiles.StartTile.A5, trajFinal.end(), parkPosition, driver);
        }
        if (parkPosition < 1){
            parkPosition = 1;
            parkingInstructions = ParkingCalculator.calculateParking("START_PARKING_SEQUENCE", StartingTiles.StartTile.A5, trajFinal.end(), parkPosition, driver);
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
