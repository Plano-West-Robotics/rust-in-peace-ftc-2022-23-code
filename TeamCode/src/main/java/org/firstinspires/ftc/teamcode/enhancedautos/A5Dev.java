package org.firstinspires.ftc.teamcode.enhancedautos;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.driveobjs.ActionObjectOld;
import org.firstinspires.ftc.teamcode.driveobjs.aprilTag.AprilTagDetector;


@Autonomous (group = "Dev")
@Config
public class A5Dev extends EnhancedAutoMode {
    FtcDashboard dashboard = FtcDashboard.getInstance();
    Telemetry dashboardTelemetry = dashboard.getTelemetry();
    public static Pose2d startingPos = new Pose2d(38, 63, Math.toRadians(270));

    public static ActionObjectOld[] actionObjectList = {
            /**
             * this section drops off inital cone
            */
            new ActionObjectOld(36, 60, 270, 0),
            new ActionObjectOld(12, 60, 270, 0),
            new ActionObjectOld(12,12, 270, 0 ),
            new ActionObjectOld(24.5,12, 270, 13),
            new ActionObjectOld(24.5,7, 270, 12),
            new ActionObjectOld(24.5,7, 270, 21),
            new ActionObjectOld(24.5,12, 270, 14 ),
            new ActionObjectOld(32,12, 270, 0 ),
            new ActionObjectOld(40,11, 0, 0 ),
            new ActionObjectOld(62, 11, 0, 20),
            new ActionObjectOld(35, 11, 0, 0),
            new ActionObjectOld(35, 11, 45, 0),
            new ActionObjectOld(50, 24, 45, 0),
            new ActionObjectOld(35, 11, 0, 0),

            /**
             * this section picks up more cones

            new ActionObject(14, 13, 0, 0),
            new ActionObject(14, 13, 0, 0),

            new ActionObject(62, 11, 0, 14),
            new ActionObject(62, 11, 0, 20),
            new ActionObject(62, 11, 0, 11),
            new ActionObject(24, 11, -90, 13),
            new ActionObject(24, 6, -90, 12),
            new ActionObject(24, 6, -90, 21),
            new ActionObject(24, 11, -90, 0),
            new ActionObject(62, 11, 0, 14),
            */
    };


    private StartTile startTile = StartTile.A5; //startingTile.[tile]
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

        initThings(startingPos, startTile, actionObjectList, parkPosition);

        run();
    }

}