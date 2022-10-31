package org.firstinspires.ftc.teamcode.enhancedautos;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.driveobjs.ActionObject;

import java.util.ArrayList;
import java.util.Arrays;

@Disabled
@Autonomous
@Config
public class AutoBase extends EnhancedAutoMode {
    FtcDashboard dashboard = FtcDashboard.getInstance();
    Telemetry dashboardTelemetry = dashboard.getTelemetry();
    public static Pose2d startingPos = new Pose2d();
    public static ActionObject[] actionObjectList = {
            new ActionObject(1, 1, 1, 1),
            new ActionObject(1, 1, 1, 1)
    };
    public static StartTile startTile = null; //startingTile.[tile]

    @Override
    public void runOpMode(){
        actionObjects = new ArrayList<ActionObject>(Arrays.asList(actionObjectList));

        int parkPosition = readAprilTag();

        initThings(startingPos);


        waitForStart();


        run(startTile, parkPosition);
    }

}
