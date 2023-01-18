package org.firstinspires.ftc.teamcode.enhancedautos;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.driveobjs.DriverMethodQueue;

@Disabled
@Autonomous
@Config

public class AutoTest extends EnhancedAutoMode {
    FtcDashboard dashboard = FtcDashboard.getInstance();
    Telemetry dashboardTelemetry = dashboard.getTelemetry();

    //edit this
    public static Pose2d startingPos = new Pose2d();

    //edit this
    public static StartTile startTile = StartTile.A2;

    //edit this
    private DriverMethodQueue queue = new DriverMethodQueue()
            .add("driveTo", new Pose2d(1, 1, 1))
            .add("driveTo");






    private int parkPosition = 0;

    @Override
    public void runOpMode(){


        while (!isStarted() && !isStopRequested()) {
            parkPosition = readAprilTag();
        }

        //initThings(startingPos, startTile, queue, parkPosition);

        run();
    }

}
