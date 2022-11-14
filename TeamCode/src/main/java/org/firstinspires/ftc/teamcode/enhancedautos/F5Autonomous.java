package org.firstinspires.ftc.teamcode.enhancedautos;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.driveobjs.ActionObject;

@Autonomous (group = "Full Parking")
@Config
public class F5Autonomous extends EnhancedAutoMode {
    FtcDashboard dashboard = FtcDashboard.getInstance();
    Telemetry dashboardTelemetry = dashboard.getTelemetry();
    public static Pose2d startingPos = new Pose2d(38, -62, Math.toRadians(90));

    public static ActionObject[] actionObjectList = {
            new ActionObject(36, -60, -90, 0),
            new ActionObject(36, -60, 180, 0),
            new ActionObject(12, -60, 180, 0),
            new ActionObject(12, -24, 180, 0),
            new ActionObject(9, -24, 180, 0),
            new ActionObject(12, -24, 180, 0)
    };


    private StartTile startTile = null; //startingTile.[tile]
    private int parkPosition = 0;

    @Override
    public void runOpMode(){

        initThings(startingPos, startTile, actionObjectList, parkPosition);

        while (!isStarted() && !isStopRequested()) {
            parkPosition = readAprilTag();
        }

        initThings(startingPos, startTile, actionObjectList, parkPosition);

        run();
    }

}
