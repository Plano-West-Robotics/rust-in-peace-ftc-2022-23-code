package org.firstinspires.ftc.teamcode.enhancedautos;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.driveobjs.ActionObject;
import org.firstinspires.ftc.teamcode.driveobjs.aprilTag.AprilTagDetector;


@Autonomous (group = "Full Parking")
@Config
public class A5Autonomous extends EnhancedAutoMode {
    FtcDashboard dashboard = FtcDashboard.getInstance();
    Telemetry dashboardTelemetry = dashboard.getTelemetry();
    public static Pose2d startingPos = new Pose2d(38, 63, Math.toRadians(270));

    public static ActionObject[] actionObjectList = {
            /**
             * this section drops off inital cone
            */
            new ActionObject(36, 60, 270, 0),
            new ActionObject(12, 60, 270, 0),
            new ActionObject(12,36, 270, 0 ),
            new ActionObject(12,36, 180, 0 ),
            new ActionObject(12, 24.5, 180, 13),
            new ActionObject(6.5, 24.5, 180, 12),
            new ActionObject(6.5, 24.5, 180, 21),
            new ActionObject(14, 24.5, 180, 0),
            new ActionObject(14, 13, 180, 0),

            /**
             * this section picks up more cones
            */
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
