package org.firstinspires.ftc.teamcode.enhancedautos;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.driveobjs.ActionObjectOld;
import org.firstinspires.ftc.teamcode.driveobjs.aprilTag.AprilTagDetector;

@Autonomous (group = "Full Parking")
@Config
public class F5Autonomous extends EnhancedAutoMode {
    FtcDashboard dashboard = FtcDashboard.getInstance();
    Telemetry dashboardTelemetry = dashboard.getTelemetry();
    public static Pose2d startingPos = new Pose2d(34, -63, Math.toRadians(90));

    public static ActionObjectOld[] actionObjectList = {
            /**
             * this section drops off inital cone
            */
            new ActionObjectOld(36, -60, 90, 0), //travels to center of tile
            new ActionObjectOld(12, -60, 90, 0),
            new ActionObjectOld(12,-36, 90, 0 ),
            new ActionObjectOld(12,-36, 180, 0 ),
            new ActionObjectOld(12, -26, 180, 13), //moves arm up
            new ActionObjectOld(5, -26, 180, 33),
            //new ActionObjectOld(5, -26, 180, 12), //moves arm down after moving in
            new ActionObjectOld(5, -26, 180, 21), //lets go of cone
            new ActionObjectOld(13, -26, 180, 0), //decouples from junction, lets the parking calc itself
            new ActionObjectOld(14, -13, 180, 0)

            /**
             * this section picks up and drops off more cones
             * TODO: True
            */

    };


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

        initThings(startingPos, startTile, actionObjectList, parkPosition);

        run();
    }

}
