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
public class F2Autonomous extends EnhancedAutoMode {
    FtcDashboard dashboard = FtcDashboard.getInstance();
    Telemetry dashboardTelemetry = dashboard.getTelemetry();
    public static Pose2d startingPos = new Pose2d(-38, -63, Math.toRadians(90));

    public static ActionObject[] actionObjectList = {
            new ActionObject(-36, -60, 90, 0),
            new ActionObject(-12, -60, 90, 0),
            new ActionObject(-12,-36, 90, 0 ),
            new ActionObject(-12,-36, 0, 0 ),
            new ActionObject(-12, -23, 0, 13),
            new ActionObject(-5, -23, 0, 12),
            new ActionObject(-5, -23, 0, 21),
            new ActionObject(-14, -23, 0, 0),
            new ActionObject(-14, -13, 0, 0)
    };


    private StartTile startTile = StartTile.F2; //startingTile.[tile]
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
