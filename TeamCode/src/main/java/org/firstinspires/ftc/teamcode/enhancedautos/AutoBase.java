package org.firstinspires.ftc.teamcode.enhancedautos;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.driveobjs.ActionObjectOld;

@Disabled
@Autonomous
@Config

public class AutoBase extends EnhancedAutoMode {
    FtcDashboard dashboard = FtcDashboard.getInstance();
    Telemetry dashboardTelemetry = dashboard.getTelemetry();
    public static Pose2d startingPos = new Pose2d();

    public static ActionObjectOld[] actionObjectList = {
            new ActionObjectOld(1, 1, 1, 1),
            new ActionObjectOld(1, 1, 1, 1)
    };


    private StartTile startTile = null; //StartTile.[tile]
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
