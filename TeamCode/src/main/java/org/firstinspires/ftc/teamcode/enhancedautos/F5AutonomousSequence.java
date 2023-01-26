package org.firstinspires.ftc.teamcode.enhancedautos;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.driveobjs.EnhancedDriver;
import org.firstinspires.ftc.teamcode.driveobjs.aprilTag.AprilTagDetector;
import org.firstinspires.ftc.teamcode.roadRunner.trajectorysequence.TrajectorySequence;

@Autonomous (group = "Full Parking")
@Config
public class F5AutonomousSequence extends EnhancedAutoMode {
    FtcDashboard dashboard = FtcDashboard.getInstance();
    Telemetry dashboardTelemetry = dashboard.getTelemetry();
    public static Pose2d startingPos = new Pose2d(30, -63, Math.toRadians(90));








    private StartTile startTile = StartTile.F5; //startingTile.[tile]
    private int parkPosition = 0;

    @Override
    public void runOpMode(){

        detector = new AprilTagDetector(hardwareMap);
        while (!isStarted() && !isStopRequested()) {
            parkPosition = detector.getPos();
            telemetry.addLine(String.format("\nDetected tag ID=%d", parkPosition));
            telemetry.update();
            sleep(50);
        }
        enhancedDriver = new EnhancedDriver(hardwareMap);
        Pose2d startPose = new Pose2d(32, -62, Math.toRadians(90));
        enhancedDriver.closeGrabber();

        enhancedDriver.setPoseEstimate(startPose);
        TrajectorySequence trajSeq = enhancedDriver.trajectorySequenceBuilder(startPose)
                .waitSeconds(2)
                .lineToConstantHeading(new Vector2d(14, -60))
                .splineToConstantHeading(new Vector2d(12, -60), Math.toRadians(90))
                .lineToConstantHeading(new Vector2d(12, -42))
                .addDisplacementMarker(()->{
                    enhancedDriver.moveArm(3);
                })
                .lineToSplineHeading(new Pose2d(12, -30, Math.toRadians(180)))
                //.lineToConstantHeading(new Vector2d(12, -24))
                .splineToConstantHeading(new Vector2d(5, -26), Math.toRadians(180))
                .addDisplacementMarker(()->{
                    enhancedDriver.moveArm(2);
                })
                .addDisplacementMarker(()->{
                    enhancedDriver.openGrabber();
                })
                .waitSeconds(1)
                .splineToConstantHeading(new Vector2d(12, -18), Math.toRadians(90))
                .splineToSplineHeading(new Pose2d(12, -12, Math.toRadians(270)), Math.toRadians(0))
                .lineTo(calculateTargetPositions(StartTile.F5, parkPosition).vec())
                .build();



        enhancedDriver.followTrajectorySequence(trajSeq);
    }

}
