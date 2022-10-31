package org.firstinspires.ftc.teamcode.enhancedautos;

import android.drm.DrmStore;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.driveobjs.ActionObject;

import java.util.ArrayList;
import java.util.List;

@Disabled
@Autonomous
@Config
public class AutoBase extends EnhancedAutoMode {
    FtcDashboard dashboard = FtcDashboard.getInstance();
    Telemetry dashboardTelemetry = dashboard.getTelemetry();
    public static ActionObject startingPos = new ActionObject(1, 1, 1);

    @Override
    public void runOpMode(){
        actionObjects = new ArrayList<ActionObject>(0);
        actionObjects.add(startingPos);
        actionObjects.add(new ActionObject(1, 1, 1, 1/*x, y, angle, methodID*/));
        //actionObjects.add(new ActionObject(1, 1, 1, 1/*x, y, angle, methodID*/));
        //actionObjects.add(new ActionObject(1, 1, 1, 1/*x, y, angle, methodID*/));
        //actionObjects.add(new ActionObject(1, 1, 1, 1/*x, y, angle, methodID*/));
        readAprilTag();


        waitForStart();
        initThings();
        run();
    }

}
